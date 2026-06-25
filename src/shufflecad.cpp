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
    std::atomic<int> camera_toggler = 0;

    void start();

    void on_out_vars();
    void on_in_vars();
    void on_chart_vars();
    void on_outcad_vars();
    void on_rpi_vars();
    void on_camera_vars();
    void on_joy_vars();
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

        socket_t client = accept(server_fd, nullptr, nullptr);
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
    camera_variables_channel = new ShflTalkPort(63254, std::bind(&ConnectionHelper::on_camera_vars, this), 0.03f, true);
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
}

void ConnectionHelper::stop() {
    out_variables_channel->stop();
    in_variables_channel->stop();
    chart_variables_channel->stop();
    outcad_variables_channel->stop();
    rpi_variables_channel->stop();
    camera_variables_channel->stop();
    joy_variables_channel->stop();
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
    std::lock_guard<std::mutex> lock(data_mutex);
    if (shufflecad->camera_variables_array.empty()) {
        camera_variables_channel->out_string = "null";
        camera_variables_channel->out_bytes = {'n','u','l','l'};
        return;
    }

    int requested_idx = -1;
    try {
        requested_idx = std::stoi(camera_variables_channel->str_from_client);
    } catch (...) { requested_idx = -1; }

    int final_idx = 0;
    if (requested_idx == -1) {
        final_idx = camera_toggler;
        camera_toggler = (camera_toggler + 1) % shufflecad->camera_variables_array.size();
    } else {
        final_idx = requested_idx % shufflecad->camera_variables_array.size();
    }

    CameraVariable* curr_var = shufflecad->camera_variables_array[final_idx];
    
    // name;width:height
    std::string shape_str = std::to_string(curr_var->width) + ":" + std::to_string(curr_var->height);
    camera_variables_channel->out_string = curr_var->name + ";" + shape_str;
    
    camera_variables_channel->out_bytes = curr_var->get_value();
}

void ConnectionHelper::on_joy_vars() {
    std::string data = joy_variables_channel->out_string;
    if (data != "null" && !data.empty()) {
        std::lock_guard<std::mutex> lock(data_mutex);
        auto string_vars = split(data, '&');
        for (const auto& item : string_vars) {
            auto parts = split(item, ';');
            if (parts.size() == 2) {
                try {
                    shufflecad->joystick_values[parts[0]] = std::stoi(parts[1]);
                } catch (...) {}
            }
        }
    }
}