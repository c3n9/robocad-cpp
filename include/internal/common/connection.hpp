#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <cstring>
#include <cstdint>

#include "robot.hpp"

#ifdef _WIN32
    #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
    #endif
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib") // for MSVC
    typedef SOCKET socket_t;
    #define INVALID_SCT INVALID_SOCKET
    #define SCT_ERROR SOCKET_ERROR
    #define SHUT_RDWR SD_BOTH
#else
    #include <sys/socket.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <netinet/in.h>
    typedef int socket_t;
    #define INVALID_SCT -1
    #define SCT_ERROR -1
    #define closesocket close
#endif

// only for windows, on linux it's no-op
class SocketInit {
public:
    SocketInit() {
#ifdef _WIN32
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
    }
    ~SocketInit() {
#ifdef _WIN32
        WSACleanup();
#endif
    }
};

// global ini
static SocketInit _g_socket_init;

// --- Helpers ---

static bool send_all(socket_t s, const void* data, size_t size) {
    const char* ptr = static_cast<const char*>(data);
    while (size > 0) {
        int sent = send(s, ptr, (int)size, 0);
        if (sent <= 0) return false;
        ptr += sent;
        size -= sent;
    }
    return true;
}

static bool recv_all(socket_t s, void* data, size_t size) {
    char* ptr = static_cast<char*>(data);
    while (size > 0) {
        int rcvd = recv(s, ptr, (int)size, 0);
        if (rcvd <= 0) return false;
        ptr += rcvd;
        size -= rcvd;
    }
    return true;
}

// --- ListenPort ---

class ListenPort {
private:
    Robot* robot;
    int port;
    socket_t sct = INVALID_SCT;
    std::atomic<bool> stop_thread{false};
    std::thread worker;
    std::mutex bytes_mutex;

public:
    std::vector<uint8_t> out_bytes;

    ListenPort(Robot* r, int p) : robot(r), port(p) {}
    ~ListenPort() { stop_listening(); }

    void start_listening() {
        stop_thread = false;
        worker = std::thread(&ListenPort::listening, this);
    }

    void listening() {
        sct = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sct == INVALID_SCT) return;

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

        if (connect(sct, (struct sockaddr*)&addr, sizeof(addr)) == SCT_ERROR) {
            robot->write_log("LP: Failed to connect on port " + std::to_string(port));
            closesocket(sct);
            sct = INVALID_SCT;
            return;
        }

        robot->write_log("LP: Connected: " + std::to_string(port));

        // UTF-16LE "Wait for data"
        std::u16string msg = u"Wait for data";
        const char* msg_raw = reinterpret_cast<const char*>(msg.c_str());
        uint32_t msg_len = (uint32_t)msg.size() * 2;

        while (!stop_thread) {
            if (!send_all(sct, &msg_len, 4)) break;
            if (!send_all(sct, msg_raw, msg_len)) break;

            uint32_t data_len = 0;
            if (!recv_all(sct, &data_len, 4)) break;

            // read
            std::vector<uint8_t> buffer(data_len);
            if (data_len > 0) {
                if (!recv_all(sct, buffer.data(), data_len)) break;
            }

            {
                std::lock_guard<std::mutex> lock(bytes_mutex);
                out_bytes = std::move(buffer);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }

        robot->write_log("LP: Disconnected: " + std::to_string(port));
        shutdown(sct, SHUT_RDWR);
        closesocket(sct);
        sct = INVALID_SCT;
    }

    void stop_listening() {
        stop_thread = true;
        if (sct != INVALID_SCT) shutdown(sct, SHUT_RDWR);
        if (worker.joinable()) worker.join();
        reset_out();
    }

    void reset_out() {
        std::lock_guard<std::mutex> lock(bytes_mutex);
        out_bytes.clear();
    }

    std::vector<uint8_t> get_bytes_safe() 
    {
        std::lock_guard<std::mutex> lock(bytes_mutex);
        return out_bytes;
    }
};

// --- TalkPort ---

class TalkPort {
private:
    Robot* robot;
    int port;
    socket_t sct = INVALID_SCT;
    std::atomic<bool> stop_thread{false};
    std::thread worker;
    std::mutex bytes_mutex;

public:
    std::vector<uint8_t> out_bytes;

    TalkPort(Robot* r, int p) : robot(r), port(p) {}
    ~TalkPort() { stop_talking(); }

    void start_talking() {
        stop_thread = false;
        worker = std::thread(&TalkPort::talking, this);
    }

    void talking() {
        sct = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sct == INVALID_SCT) return;

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

        if (connect(sct, (struct sockaddr*)&addr, sizeof(addr)) == SCT_ERROR) {
            robot->write_log("TP: Failed to connect on port " + std::to_string(port));
            closesocket(sct);
            sct = INVALID_SCT;
            return;
        }

        robot->write_log("TP: Connected: " + std::to_string(port));

        while (!stop_thread) {
            std::vector<uint8_t> to_send;
            {
                std::lock_guard<std::mutex> lock(bytes_mutex);
                to_send = out_bytes;
            }

            uint32_t len = (uint32_t)to_send.size();
            if (!send_all(sct, &len, 4)) break;
            if (len > 0) {
                if (!send_all(sct, to_send.data(), len)) break;
            }

            // server replies with two 4-byte acks (8 bytes total); read them fully
            char dummy[8];
            if (!recv_all(sct, dummy, 8)) break;

            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }

        robot->write_log("TP: Disconnected: " + std::to_string(port));
        shutdown(sct, SHUT_RDWR);
        closesocket(sct);
        sct = INVALID_SCT;
    }

    void stop_talking() {
        stop_thread = true;
        if (sct != INVALID_SCT) shutdown(sct, SHUT_RDWR);
        if (worker.joinable()) worker.join();
    }

    void set_bytes_safe(std::vector<uint8_t> bytes) 
    {
        std::lock_guard<std::mutex> lock(bytes_mutex);
        out_bytes = std::move(bytes);
    }
};