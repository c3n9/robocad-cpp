#include "shufflecad.hpp"

#include <csignal>
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <cstring>
#include <cstdint>

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

class Shufflecad;
class SocketInit;
class ShflListenPort;
class ShflTalkPort;

class ConnectionHelper
{
public:
    ConnectionHelper(Shufflecad* shufflecad, Robot* robot);
    ~ConnectionHelper();

    void stop();

    std::mutex data_mutex; // for variables_array and print_array safety

private:
    Shufflecad* shufflecad;
    Robot* robot;

    ShflTalkPort* out_variables_channel;
    ShflListenPort* in_variables_channel;
    ShflTalkPort* chart_variables_channel;
    ShflTalkPort* outcad_variables_channel;
    ShflTalkPort* rpi_variables_channel;
    ShflTalkPort* camera_variables_channel;
    ShflListenPort* joy_variables_channel;

    SocketInit net_guard;

    // Камеры: список (имя;ширина:высота) идёт по TCP, кадры — по UDP чанками.
    static constexpr int CAMERA_UDP_PORT = 63260;
    static constexpr int CAMERA_UDP_CHUNK = 1400;
    socket_t camera_udp_sock = INVALID_SCT;
    std::thread camera_udp_thread;
    std::atomic<bool> stop_camera_udp{false};
    std::atomic<int> selected_camera{0};
    uint32_t camera_frame_id = 0;

    void start();

    void on_out_vars();
    void on_in_vars();
    void on_chart_vars();
    void on_outcad_vars();
    void on_rpi_vars();
    void on_camera_vars();
    void on_joy_vars();

    void start_camera_udp();
    void stop_camera_udp_thread();
    void camera_udp_loop();
    void send_frame_udp(const std::string& target, uint16_t camera_index, const std::vector<uint8_t>& data);
};

// global ini
static SocketInit _g_socket_init;

static Shufflecad* current_instance = nullptr;
static void handler(int signum)
{
    current_instance->robot->write_log("Program stopped from handler");
    current_instance->robot->write_log("Signal handler called with signal " + std::to_string(signum));
    current_instance->stop();
    exit(signum);
}

Shufflecad::Shufflecad(Robot* robot) : robot(robot) 
{
    // Set the global instance pointer for signal handling
    current_instance = this;
    
    signal(SIGINT, handler);
    signal(SIGTERM, handler);

    connection_helper = new ConnectionHelper(this, robot);
}

Shufflecad::~Shufflecad()
{
    // delete connection helper
    delete connection_helper;
    connection_helper = NULL;
}

void Shufflecad::stop() 
{
    // stop connection helper
    connection_helper->stop();
}

ShuffleVariable* Shufflecad::add_var(ShuffleVariable* var)
{
    std::lock_guard<std::mutex> lock(connection_helper->data_mutex);
    variables_array.push_back(var);
    return var;
}

CameraVariable* Shufflecad::add_var(CameraVariable* var)
{
    std::lock_guard<std::mutex> lock(connection_helper->data_mutex);
    camera_variables_array.push_back(var);
    return var;
}

void Shufflecad::print_to_log(std::string message, std::string message_type, std::string color)
{
    std::lock_guard<std::mutex> lock(connection_helper->data_mutex);
    std::string formatted_message = message_type + "@" + message + color;
    print_array.push_back(formatted_message);
}

std::vector<std::string> Shufflecad::get_print_array()
{
    return print_array;
}
void Shufflecad::clear_print_array()
{
    std::lock_guard<std::mutex> lock(connection_helper->data_mutex);
    print_array.clear();
}

// ------------- Helper functions --------------

std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

template <typename T>
std::string join(const std::vector<T>& elements, const std::string& delimiter) {
    std::ostringstream os;
    for (size_t i = 0; i < elements.size(); ++i) {
        os << elements[i];
        if (i != elements.size() - 1) os << delimiter;
    }
    return os.str();
}

class SplitFrames {
    socket_t sock_fd;
public:
    SplitFrames(socket_t fd) : sock_fd(fd) {}

    bool write_data(const std::vector<uint8_t>& data) {
        uint32_t size = static_cast<uint32_t>(data.size());
        if (send(sock_fd, (const char*)&size, 4, 0) <= 0) return false;
        if (send(sock_fd, (const char*)data.data(), size, 0) <= 0) return false;
        return true;
    }

    bool write_string(const std::string& s) {
        std::vector<uint8_t> data(s.begin(), s.end());
        return write_data(data);
    }

    std::vector<uint8_t> read_data() {
        uint32_t size = 0;
        int res = recv(sock_fd, (char*)&size, 4, 0);
        if (res <= 0) return {};

        std::vector<uint8_t> buffer(size);
        uint32_t total_read = 0;
        while (total_read < size) {
            int n = recv(sock_fd, (char*)buffer.data() + total_read, size - total_read, 0);
            if (n <= 0) break;
            total_read += n;
        }
        return buffer;
    }

    std::string read_string() {
        auto data = read_data();
        if (data.empty()) return "null";
        return std::string(data.begin(), data.end());
    }
};

// ------------- Talk and Listen ports --------------

class BasePort {
protected:
    int port;
    std::atomic<bool> stop_thread{false};
    std::thread worker;
    socket_t server_fd = INVALID_SCT;
    float delay;
    std::function<void()> event_handler;

public:
    std::string out_string = "null";
    std::vector<uint8_t> out_bytes;
    std::string str_from_client = "-1";

    std::string client_address;

    BasePort(int p, std::function<void()> handler, float d)
        : port(p), event_handler(handler), delay(d) {}

    virtual ~BasePort() { stop(); }

    void stop() {
        stop_thread = true;
        if (server_fd != INVALID_SCT) {
            shutdown(server_fd, SHUT_RDWR);
            closesocket(server_fd);
            server_fd = INVALID_SCT;
        }
        if (worker.joinable()) worker.join();
    }

    socket_t wait_for_connection() {
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd == INVALID_SCT) return INVALID_SCT;

        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));

        sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);

        if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) == SCT_ERROR) return INVALID_SCT;
        listen(server_fd, 1);

        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        socket_t client = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
        if (client != INVALID_SCT) {
            char ip_buf[INET_ADDRSTRLEN];
            if (inet_ntop(AF_INET, &client_addr.sin_addr, ip_buf, sizeof(ip_buf)) != nullptr)
                client_address = ip_buf;
        }
        return client;
    }
};

class ShflTalkPort : public BasePort {
    bool is_camera;
public:
    ShflTalkPort(int p, std::function<void()> h, float d, bool cam = false)
        : BasePort(p, h, d), is_camera(cam) {}

    void start_talking() {
        worker = std::thread([this]() {
            socket_t client_fd = wait_for_connection();
            if (client_fd == INVALID_SCT) return;

            SplitFrames handler(client_fd);
            while (!stop_thread) {
                if (event_handler) event_handler();

                if (is_camera) {
                    if (!handler.write_string(out_string)) break;
                    handler.read_data();
                    if (!handler.write_data(out_bytes)) break;
                    str_from_client = handler.read_string();
                } else {
                    if (!handler.write_string(out_string)) break;
                    str_from_client = handler.read_string();
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay * 1000)));
            }
            closesocket(client_fd);
        });
    }
};

class ShflListenPort : public BasePort {
public:
    ShflListenPort(int p, std::function<void()> h, float d) : BasePort(p, h, d) {}

    void start_listening() {
        worker = std::thread([this]() {
            socket_t client_fd = wait_for_connection();
            if (client_fd == INVALID_SCT) return;

            SplitFrames handler(client_fd);
            while (!stop_thread) {
                if (!handler.write_string("Waiting for data")) break;

                out_string = handler.read_string();
                if (event_handler) event_handler();

                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay * 1000)));
            }
            closesocket(client_fd);
        });
    }
};

// ------------- Connection helper --------------


ConnectionHelper::ConnectionHelper(Shufflecad* sc, Robot* r) : shufflecad(sc), robot(r) {
    SocketInit net_guard;
    
    out_variables_channel = new ShflTalkPort(63253, std::bind(&ConnectionHelper::on_out_vars, this), 0.004f);
    in_variables_channel = new ShflListenPort(63258, std::bind(&ConnectionHelper::on_in_vars, this), 0.004f);
    chart_variables_channel = new ShflTalkPort(63255, std::bind(&ConnectionHelper::on_chart_vars, this), 0.002f);
    outcad_variables_channel = new ShflTalkPort(63257, std::bind(&ConnectionHelper::on_outcad_vars, this), 0.1f);
    rpi_variables_channel = new ShflTalkPort(63256, std::bind(&ConnectionHelper::on_rpi_vars, this), 0.5f);
    camera_variables_channel = new ShflTalkPort(63254, std::bind(&ConnectionHelper::on_camera_vars, this), 0.03f);
    joy_variables_channel = new ShflListenPort(63259, std::bind(&ConnectionHelper::on_joy_vars, this), 0.004f);

    this->start();
}

ConnectionHelper::~ConnectionHelper() {
    delete out_variables_channel;
    delete in_variables_channel;
    delete chart_variables_channel;
    delete outcad_variables_channel;
    delete rpi_variables_channel;
    delete camera_variables_channel;
    delete joy_variables_channel;
    out_variables_channel = nullptr;
    in_variables_channel = nullptr;
    chart_variables_channel = nullptr;
    outcad_variables_channel = nullptr;
    rpi_variables_channel = nullptr;
    camera_variables_channel = nullptr;
    joy_variables_channel = nullptr;
}

void ConnectionHelper::start()
{
    out_variables_channel->start_talking();
    in_variables_channel->start_listening();
    chart_variables_channel->start_talking();
    outcad_variables_channel->start_talking();
    rpi_variables_channel->start_talking();
    camera_variables_channel->start_talking();
    joy_variables_channel->start_listening();
    start_camera_udp();
}

void ConnectionHelper::stop() {
    out_variables_channel->stop();
    in_variables_channel->stop();
    chart_variables_channel->stop();
    outcad_variables_channel->stop();
    rpi_variables_channel->stop();
    camera_variables_channel->stop();
    joy_variables_channel->stop();
    stop_camera_udp_thread();
}

void ConnectionHelper::on_out_vars() {
    std::lock_guard<std::mutex> lock(data_mutex);
    std::vector<std::string> segments;
    
    for (ShuffleVariable* var : shufflecad->variables_array) {
        if (var->type != ShuffleVariable::CHART_TYPE) {
            // name;value;type;direction
            std::string s = var->name + ";" + var->value + ";" + var->type + ";" + var->direction;
            segments.push_back(s);
        }
    }

    out_variables_channel->out_string = segments.empty() ? "null" : join(segments, "&");
}

void ConnectionHelper::on_in_vars() {
    std::string data = in_variables_channel->out_string;
    if (data != "null" && !data.empty()) {
        std::lock_guard<std::mutex> lock(data_mutex);
        auto string_vars = split(data, '&');
        for (const auto& item : string_vars) {
            auto parts = split(item, ';');
            if (parts.size() == 2) {
                std::string name = parts[0];
                std::string value = parts[1];
                for (ShuffleVariable* var : shufflecad->variables_array) {
                    if (var->name == name) {
                        var->value = value;
                        break;
                    }
                }
            }
        }
    }
}

void ConnectionHelper::on_chart_vars() {
    std::lock_guard<std::mutex> lock(data_mutex);
    std::vector<std::string> segments;
    for (ShuffleVariable* var : shufflecad->variables_array) {
        if (var->type == ShuffleVariable::CHART_TYPE) {
            segments.push_back(var->name + ";" + var->value);
        }
    }
    chart_variables_channel->out_string = segments.empty() ? "null" : join(segments, "&");
}

void ConnectionHelper::on_outcad_vars() {
    std::lock_guard<std::mutex> lock(data_mutex);
    if (!shufflecad->print_array.empty()) {
        outcad_variables_channel->out_string = join(shufflecad->print_array, "&");
        shufflecad->print_array.clear();  // clear directly; clear_print_array() would re-lock data_mutex
    } else {
        outcad_variables_channel->out_string = "null";
    }
}

void ConnectionHelper::on_rpi_vars() {
    std::vector<std::string> stats = {
        std::to_string(robot->robot_info->temperature),
        std::to_string(robot->robot_info->memory_load),
        std::to_string(robot->robot_info->cpu_load),
        std::to_string(robot->power),
        std::to_string(robot->robot_info->spi_time_dev),
        std::to_string(robot->robot_info->rx_spi_time_dev),
        std::to_string(robot->robot_info->tx_spi_time_dev),
        std::to_string(robot->robot_info->spi_count_dev),
        std::to_string(robot->robot_info->com_time_dev),
        std::to_string(robot->robot_info->rx_com_time_dev),
        std::to_string(robot->robot_info->tx_com_time_dev),
        std::to_string(robot->robot_info->com_count_dev)
    };
    rpi_variables_channel->out_string = join(stats, "&");
}

void ConnectionHelper::on_camera_vars() {
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        if (shufflecad->camera_variables_array.empty()) {
            camera_variables_channel->out_string = "null";
        } else {
            std::vector<std::string> segments;
            for (CameraVariable* c : shufflecad->camera_variables_array) {
                segments.push_back(c->name + ";" + std::to_string(c->width.load()) + ":" + std::to_string(c->height.load()));
            }
            camera_variables_channel->out_string = join(segments, "&");
        }
    }

    try {
        selected_camera = std::stoi(camera_variables_channel->str_from_client);
    } catch (...) { }
}

void ConnectionHelper::start_camera_udp() {
    stop_camera_udp = false;
    camera_udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    camera_udp_thread = std::thread(&ConnectionHelper::camera_udp_loop, this);
}

void ConnectionHelper::stop_camera_udp_thread() {
    stop_camera_udp = true;
    if (camera_udp_sock != INVALID_SCT) {
        closesocket(camera_udp_sock);
        camera_udp_sock = INVALID_SCT;
    }
    if (camera_udp_thread.joinable()) camera_udp_thread.join();
}


void ConnectionHelper::camera_udp_loop() {
    long diag = 0;
    while (!stop_camera_udp) {
        try {
            CameraVariable* cam = nullptr;
            int idx = selected_camera;
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                if (!shufflecad->camera_variables_array.empty()) {
                    if (idx < 0 || idx >= (int)shufflecad->camera_variables_array.size()) idx = 0;
                    cam = shufflecad->camera_variables_array[idx];
                }
            }

            std::string target = camera_variables_channel->client_address;
            size_t data_size = 0; bool is_jpeg = false; bool sent = false;
            if (cam != nullptr && !target.empty() && cam->width > 0 && cam->height > 0) {
                std::vector<uint8_t> data = cam->get_value();
                data_size = data.size();
                is_jpeg = data.size() >= 2 && data[0] == 0xFF && data[1] == 0xD8;
                if (is_jpeg) { send_frame_udp(target, (uint16_t)idx, data); sent = true; }
            }
        } catch (...) { }

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

void ConnectionHelper::send_frame_udp(const std::string& target, uint16_t camera_index, const std::vector<uint8_t>& data) {
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(CAMERA_UDP_PORT);
    if (inet_pton(AF_INET, target.c_str(), &dst.sin_addr) != 1) return;

    uint32_t frame_id = camera_frame_id++;
    int total = (int)((data.size() + CAMERA_UDP_CHUNK - 1) / CAMERA_UDP_CHUNK);
    if (total < 1) total = 1;
    if (total > 0xFFFF) return; 

    for (int i = 0; i < total; i++) {
        size_t off = (size_t)i * CAMERA_UDP_CHUNK;
        size_t remaining = data.size() - off;
        size_t len = remaining < (size_t)CAMERA_UDP_CHUNK ? remaining : (size_t)CAMERA_UDP_CHUNK;
        std::vector<uint8_t> pkt(10 + len);
        pkt[0] = (uint8_t)(frame_id & 0xFF);
        pkt[1] = (uint8_t)((frame_id >> 8) & 0xFF);
        pkt[2] = (uint8_t)((frame_id >> 16) & 0xFF);
        pkt[3] = (uint8_t)((frame_id >> 24) & 0xFF);
        pkt[4] = (uint8_t)(camera_index & 0xFF);
        pkt[5] = (uint8_t)((camera_index >> 8) & 0xFF);
        pkt[6] = (uint8_t)(i & 0xFF);
        pkt[7] = (uint8_t)((i >> 8) & 0xFF);
        pkt[8] = (uint8_t)(total & 0xFF);
        pkt[9] = (uint8_t)((total >> 8) & 0xFF);
        std::memcpy(pkt.data() + 10, data.data() + off, len);
        sendto(camera_udp_sock, (const char*)pkt.data(), (int)pkt.size(), 0, (struct sockaddr*)&dst, sizeof(dst));
    }
}

void ConnectionHelper::on_joy_vars() {
    std::string data = joy_variables_channel->out_string;
    if (data != "null" && !data.empty()) {
        std::lock_guard<std::mutex> lock(data_mutex);
        auto string_vars = split(data, '&');
        for (const auto& item : string_vars) {
            auto parts = split(item, ';');
            if (parts.size() != 2) continue;

            int val;
            try {
                val = std::stoi(parts[1]);
            } catch (...) {
                continue;
            }

            const std::string& key = parts[0];
            JoystickData& joy = shufflecad->joystick_data;

            if (key == "A") joy.btn_a = val == 1;
            else if (key == "B") joy.btn_b = val == 1;
            else if (key == "X") joy.btn_x = val == 1;
            else if (key == "Y") joy.btn_y = val == 1;
            else if (key == "RightShoulder") joy.right_shoulder = val == 1;
            else if (key == "LeftShoulder") joy.left_shoulder = val == 1;
            else if (key == "DPad_Up") joy.dpad_up = val == 1;
            else if (key == "DPad_Down") joy.dpad_down = val == 1;
            else if (key == "DPad_Right") joy.dpad_right = val == 1;
            else if (key == "DPad_Left") joy.dpad_left = val == 1;
            else if (key == "LeftTrigger") joy.left_trigger = static_cast<uint8_t>(val);
            else if (key == "RightTrigger") joy.right_trigger = static_cast<uint8_t>(val);
            else if (key == "LeftThumbstick_X") joy.left_stick_x = val;
            else if (key == "LeftThumbstick_Y") joy.left_stick_y = val;
            else if (key == "RightThumbstick_X") joy.right_stick_x = val;
            else if (key == "RightThumbstick_Y") joy.right_stick_y = val;
        }
    }
}