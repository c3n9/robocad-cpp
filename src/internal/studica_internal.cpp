#include "studica_internal.hpp"

#include "common/connection_real.hpp"
#include "common/connection_sim.hpp"

#include <chrono>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>


// --- Helpers ---

static bool access_bit(uint8_t byte, int bit_pos) 
{
    return (byte >> bit_pos) & 1;
}

static long get_time_units() {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(now).count() / 100;
}

// --- RobocadConnection ---

#pragma pack(push, 1)
// Struct for Robocad: 14 float (56 bytes)
struct RobocadTxPacket {
    float speeds[4];
    float hcdio[10];
};

// Struct for Robocad: <4i2f4Hf16B (52 bytes)
struct RobocadRxPacket {
    int32_t encoders[4];
    float ultrasound[2];
    uint16_t analog[4];
    float yaw;
    uint8_t flex_and_limits[16]; 
};
#pragma pack(pop)

class RobocadConnectionStudica {
private:
    std::thread update_thread;
    std::atomic<bool> stop_thread{false};
    std::atomic<bool> stopped{false};
    StudicaInternal* robot_internal;
    Robot* robot;
    ConnectionSim* connection;

    // Build the 56-byte TX packet from the latest (atomic) robot state.
    std::vector<uint8_t> build_tx() {
        RobocadTxPacket tx;
        tx.speeds[0] = robot_internal->speed_motor_0;
        tx.speeds[1] = robot_internal->speed_motor_1;
        tx.speeds[2] = robot_internal->speed_motor_2;
        tx.speeds[3] = robot_internal->speed_motor_3;
        for (int i = 0; i < 10; i++) {
            tx.hcdio[i] = robot_internal->hcdio_values[i];
        }
        return std::vector<uint8_t>(reinterpret_cast<uint8_t*>(&tx), reinterpret_cast<uint8_t*>(&tx) + sizeof(tx));
    }

public:
    ~RobocadConnectionStudica() { stop(); }

    void start(ConnectionSim* conn, Robot* r, StudicaInternal* i) {
        this->connection = conn;
        this->robot = r;
        this->robot_internal = i;

        robot->power = 12.0; // todo: control from ConnectionSim from robocad

        stop_thread = false;
        update_thread = std::thread(&RobocadConnectionStudica::update_loop, this);
    }

    void stop() {
        if (stopped.exchange(true)) return;
        stop_thread = true;
        if (update_thread.joinable()) update_thread.join();
    }

    void update_loop() {
        while (!stop_thread) {
            // SET DATA
            connection->set_data(build_tx());

            // GET DATA
            auto raw_data = connection->get_data();
            if (raw_data.size() >= 52) {
                RobocadRxPacket* rx = (RobocadRxPacket*)raw_data.data();
                robot_internal->enc_motor_0 = rx->encoders[0];
                robot_internal->enc_motor_1 = rx->encoders[1];
                robot_internal->enc_motor_2 = rx->encoders[2];
                robot_internal->enc_motor_3 = rx->encoders[3];
                robot_internal->ultrasound_1 = rx->ultrasound[0];
                robot_internal->ultrasound_2 = rx->ultrasound[1];
                robot_internal->analog_1 = rx->analog[0];
                robot_internal->analog_2 = rx->analog[1];
                robot_internal->analog_3 = rx->analog[2];
                robot_internal->analog_4 = rx->analog[3];
                robot_internal->yaw = rx->yaw;

                robot_internal->limit_h_0 = rx->flex_and_limits[0] == 1;
                robot_internal->limit_l_0 = rx->flex_and_limits[1] == 1;
                robot_internal->limit_h_1 = rx->flex_and_limits[2] == 1;
                robot_internal->limit_l_1 = rx->flex_and_limits[3] == 1;
                robot_internal->limit_h_2 = rx->flex_and_limits[4] == 1;
                robot_internal->limit_l_2 = rx->flex_and_limits[5] == 1;
                robot_internal->limit_h_3 = rx->flex_and_limits[6] == 1;
                robot_internal->limit_l_3 = rx->flex_and_limits[7] == 1;

                robot_internal->flex_0 = rx->flex_and_limits[8] == 1;
                robot_internal->flex_1 = rx->flex_and_limits[9] == 1;
                robot_internal->flex_2 = rx->flex_and_limits[10] == 1;
                robot_internal->flex_3 = rx->flex_and_limits[11] == 1;
                robot_internal->flex_4 = rx->flex_and_limits[12] == 1;
                robot_internal->flex_5 = rx->flex_and_limits[13] == 1;
                robot_internal->flex_6 = rx->flex_and_limits[14] == 1;
                robot_internal->flex_7 = rx->flex_and_limits[15] == 1;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
    }
};

// --- TitanCOM ---

class TitanCOMStudica {
private:
    std::thread th;
    std::atomic<bool> stop_th{false};
    ConnectionReal* connection;
    Robot* robot;
    StudicaInternal* robot_internal;
    DefaultStudicaConfiguration* conf;

public:
    TitanCOMStudica() : connection(nullptr), robot(nullptr), robot_internal(nullptr), conf(nullptr) {}
    ~TitanCOMStudica() { stop(); }

    void start_com(ConnectionReal* conn, Robot* rb, StudicaInternal* internal, DefaultStudicaConfiguration* cfg) {
        connection = conn;
        robot = rb;
        robot_internal = internal;
        conf = cfg;
        stop_th = false;
        th = std::thread(&TitanCOMStudica::com_loop, this);
    }

    void stop() {
        stop_th = true;
        if (th.joinable()) th.join();
    }

private:
    void com_loop() {
        try {
            if (connection->com_ini(conf->titan_port, conf->titan_baud) != 0) {
                robot->write_log("Failed to open COM");
                return;
            }

            long start_time = get_time_units();
            auto send_count_time = std::chrono::steady_clock::now();
            int comm_counter = 0;

            while (!stop_th) {
                long tx_time_start = get_time_units();
                std::vector<uint8_t> tx_data = set_up_tx_data();
                robot->robot_info->tx_com_time_dev = get_time_units() - tx_time_start;

                std::vector<uint8_t> rx_data = connection->com_rw(tx_data);

                long rx_time_start = get_time_units();
                set_up_rx_data(rx_data);
                robot->robot_info->rx_com_time_dev = get_time_units() - rx_time_start;

                comm_counter++;
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - send_count_time).count() >= 1) {
                    send_count_time = now;
                    robot->robot_info->com_count_dev = comm_counter;
                    comm_counter = 0;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                robot->robot_info->com_time_dev = get_time_units() - start_time;
                start_time = get_time_units();
            }
        } catch (const std::exception& e) {
            connection->com_stop();
            robot->write_log("Exception in TitanCOM: " + std::string(e.what()));
        }
    }

    void set_up_rx_data(const std::vector<uint8_t>& data) {
        if (data.size() < 43) return;
        if (data[42] != 33) {
            if (data[0] == 1) {
                if (data[24] == 111) {
                    int raw_enc_0 = (data[2] << 8) | data[1];
                    int raw_enc_1 = (data[4] << 8) | data[3];
                    int raw_enc_2 = (data[6] << 8) | data[5];
                    int raw_enc_3 = (data[8] << 8) | data[7];
                    set_up_encoders(raw_enc_0, raw_enc_1, raw_enc_2, raw_enc_3);

                    robot_internal->limit_l_0 = access_bit(data[9], 1);
                    robot_internal->limit_h_0 = access_bit(data[9], 2);
                    robot_internal->limit_l_1 = access_bit(data[9], 3);
                    robot_internal->limit_h_1 = access_bit(data[9], 4);
                    robot_internal->limit_l_2 = access_bit(data[9], 5);
                    robot_internal->limit_h_2 = access_bit(data[9], 6);
                    robot_internal->limit_l_3 = access_bit(data[10], 1);
                    robot_internal->limit_h_3 = access_bit(data[10], 2);
                }
            }
        } else {
            robot->write_log("received wrong data");
        }
    }

    std::vector<uint8_t> set_up_tx_data() {
        std::vector<uint8_t> tx_data(48, 0);
        tx_data[0] = 1;

        auto pack_speed = [&](double speed, int idx_low, int idx_high) {
            uint16_t val = static_cast<uint16_t>(std::abs(speed / 100.0 * 65535.0));
            tx_data[idx_low] = (val >> 8) & 0xFF;
            tx_data[idx_high] = val & 0xFF;
        };

        pack_speed(robot_internal->speed_motor_0, 2, 3);
        pack_speed(robot_internal->speed_motor_1, 4, 5);
        pack_speed(robot_internal->speed_motor_2, 6, 7);
        pack_speed(robot_internal->speed_motor_3, 8, 9);

        uint8_t dir_byte = 0b10000001;
        if (robot_internal->speed_motor_0 >= 0) dir_byte |= (1 << 6);
        if (robot_internal->speed_motor_1 >= 0) dir_byte |= (1 << 5);
        if (robot_internal->speed_motor_2 >= 0) dir_byte |= (1 << 4);
        if (robot_internal->speed_motor_3 >= 0) dir_byte |= (1 << 3);
        tx_data[10] = dir_byte;

        // third bit is for ProgramIsRunning
        tx_data[11] = 0b10100001;
        tx_data[20] = 222;

        return tx_data;
    }

    void set_up_encoders(int enc_0, int enc_1, int enc_2, int enc_3) {
        robot_internal->enc_motor_0 -= get_normal_diff(enc_0, robot_internal->raw_enc_motor_0);
        robot_internal->enc_motor_1 -= get_normal_diff(enc_1, robot_internal->raw_enc_motor_1);
        robot_internal->enc_motor_2 -= get_normal_diff(enc_2, robot_internal->raw_enc_motor_2);
        robot_internal->enc_motor_3 -= get_normal_diff(enc_3, robot_internal->raw_enc_motor_3);

        robot_internal->raw_enc_motor_0 = enc_0;
        robot_internal->raw_enc_motor_1 = enc_1;
        robot_internal->raw_enc_motor_2 = enc_2;
        robot_internal->raw_enc_motor_3 = enc_3;
    }

    static int get_normal_diff(int curr, int last) {
        int diff = curr - last;
        if (diff > 30000) diff = -(last + (65535 - curr));
        else if (diff < -30000) diff = curr + (65535 - last);
        return diff;
    }
};

class VMXSPIStudica {
private:
    std::thread th;
    std::atomic<bool> stop_th{false};
    ConnectionReal* connection;
    Robot* robot;
    StudicaInternal* robot_internal;
    DefaultStudicaConfiguration* conf;
    int toggler = 0;

public:
    VMXSPIStudica() : connection(nullptr), robot(nullptr), robot_internal(nullptr), conf(nullptr) {}
    ~VMXSPIStudica() { stop(); }

    void start_spi(ConnectionReal* conn, Robot* rb, StudicaInternal* internal, DefaultStudicaConfiguration* cfg) {
        connection = conn;
        robot = rb;
        robot_internal = internal;
        conf = cfg;
        toggler = 0;
        stop_th = false;
        th = std::thread(&VMXSPIStudica::spi_loop, this);
    }

    void stop() {
        stop_th = true;
        if (th.joinable()) th.join();
    }

private:
    void spi_loop() {
        try {
            if (connection->spi_ini(conf->vmx_port, conf->vmx_ch, conf->vmx_speed, conf->vmx_mode) != 0) {
                robot->write_log("Failed to open SPI");
                return;
            }

            long start_time = get_time_units();
            auto send_count_time = std::chrono::steady_clock::now();
            int comm_counter = 0;

            while (!stop_th) {
                long tx_time_start = get_time_units();
                std::vector<uint8_t> tx_list = set_up_tx_data();
                robot->robot_info->tx_spi_time_dev = get_time_units() - tx_time_start;

                std::vector<uint8_t> rx_list = connection->spi_rw(tx_list);

                long rx_time_start = get_time_units();
                set_up_rx_data(rx_list);
                robot->robot_info->rx_spi_time_dev = get_time_units() - rx_time_start;

                comm_counter++;
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - send_count_time).count() >= 1) {
                    send_count_time = now;
                    robot->robot_info->spi_count_dev = comm_counter;
                    comm_counter = 0;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                robot->robot_info->spi_time_dev = get_time_units() - start_time;
                start_time = get_time_units();
            }
        } catch (const std::exception& e) {
            connection->spi_stop();
            robot->write_log("Exception in VMXSPI: " + std::string(e.what()));
        }
    }

    void set_up_rx_data(const std::vector<uint8_t>& data) {
        if (data.empty()) return;
        if (data[0] == 1) {
            int yaw_ui = (data[2] << 8) | data[1];
            int us1_ui = (data[4] << 8) | data[3];
            robot_internal->ultrasound_1 = us1_ui / 100.0;
            int us2_ui = (data[6] << 8) | data[5];
            robot_internal->ultrasound_2 = us2_ui / 100.0;

            double power = ((data[8] << 8) | data[7]) / 100.0;
            robot->power = power;

            // calc yaw unlim
            double new_yaw = (yaw_ui / 100.0) * (access_bit(data[9], 1) ? 1.0 : -1.0);
            calc_yaw_unlim(new_yaw, robot_internal->yaw);
            robot_internal->yaw = new_yaw;

            robot_internal->flex_0 = access_bit(data[9], 2);
            robot_internal->flex_1 = access_bit(data[9], 3);
            robot_internal->flex_2 = access_bit(data[9], 4);
            robot_internal->flex_3 = access_bit(data[9], 5);
            robot_internal->flex_4 = access_bit(data[9], 6);
        } else if (data[0] == 2) {
            robot_internal->analog_1 = (data[2] << 8) | data[1];
            robot_internal->analog_2 = (data[4] << 8) | data[3];
            robot_internal->analog_3 = (data[6] << 8) | data[5];
            robot_internal->analog_4 = (data[8] << 8) | data[7];

            robot_internal->flex_5 = access_bit(data[9], 1);
            robot_internal->flex_6 = access_bit(data[9], 2);
            robot_internal->flex_7 = access_bit(data[9], 3);
        }
    }

    std::vector<uint8_t> set_up_tx_data() {
        std::vector<uint8_t> tx_list(10, 0);
        if (toggler == 0) {
            tx_list[0] = 1;
            tx_list[9] = 222;
        }
        return tx_list;
    }

    void calc_yaw_unlim(double new_yaw, double old_yaw) {
        double delta_yaw = new_yaw - old_yaw;
        if (delta_yaw < -180.0) {
            delta_yaw = (180.0 - old_yaw) + (180.0 + new_yaw);
        } else if (delta_yaw > 180.0) {
            delta_yaw = -(180.0 + old_yaw) - (180.0 - new_yaw);
        }
        robot_internal->yaw_unlim = robot_internal->yaw_unlim + delta_yaw;
    }
};

// ------------- StudicaInternal -------------

StudicaInternal::StudicaInternal(Robot* robot, RobotConfiguration* conf) : robot(robot)
{
    titan_com = nullptr;
    vmx_spi = nullptr;
    robocad_connection = nullptr;
    updater = nullptr;

    if (robot->on_real_robot)
    {
        updater = new RpiUpdater(robot);
        connection = new ConnectionReal(robot, updater, conf);
        titan_com = new TitanCOMStudica();
        titan_com->start_com((ConnectionReal*)connection, robot, this, (DefaultStudicaConfiguration*)conf);
        vmx_spi = new VMXSPIStudica();
        vmx_spi->start_spi((ConnectionReal*)connection, robot, this, (DefaultStudicaConfiguration*)conf);
    } 
    else 
    {
        connection = new ConnectionSim(robot);
        robocad_connection = new RobocadConnectionStudica();
        robocad_connection->start((ConnectionSim*)connection, robot, this);
    }
}

StudicaInternal::~StudicaInternal()
{
    // stop worker threads BEFORE deleting the connection they use
    stop();

    if (robot->on_real_robot)
    {
        delete titan_com;
        titan_com = NULL;
        delete vmx_spi;
        vmx_spi = NULL;
    }
    else
    {
        delete robocad_connection;
        robocad_connection = NULL;
    }

    delete connection;
    connection = NULL;
    delete updater;
    updater = NULL;
}

void StudicaInternal::stop()
{
    // stop worker threads first so they no longer touch the connection
    if (robot->on_real_robot)
    {
        if (titan_com) titan_com->stop();
        if (vmx_spi) vmx_spi->stop();
    }
    else
    {
        if (robocad_connection) robocad_connection->stop();
    }
    if (connection) connection->stop();
}

cv::Mat StudicaInternal::get_camera() 
{
    return connection->get_camera();
}

std::vector<float> StudicaInternal::get_lidar() 
{
    return connection->get_lidar();
}

void StudicaInternal::set_servo_angle(float angle, int pin)
{
    double dut = 0.000666 * angle + 0.05;
    hcdio_values[pin] = dut;
    std::string cmd = std::to_string(HCDIO_CONST_ARRAY[pin]) + "=" + std::to_string(dut);
    echo_to_file(cmd);
}
void StudicaInternal::set_led_state(bool state, int pin)
{
    hcdio_values[pin] = state ? 0.2f : 0.0f;
    std::string cmd = std::to_string(HCDIO_CONST_ARRAY[pin]) + "=" + std::to_string(state ? 1.0f : 0.0f);
    echo_to_file(cmd);
}
void StudicaInternal::set_servo_pwm(float pwm, int pin)
{
    double dut = pwm;
    hcdio_values[pin] = dut;
    std::string cmd = std::to_string(HCDIO_CONST_ARRAY[pin]) + "=" + std::to_string(dut);
    echo_to_file(cmd);
}
void StudicaInternal::disable_servo(int pin)
{
    hcdio_values[pin] = 0.0f;
    std::string cmd = std::to_string(HCDIO_CONST_ARRAY[pin]) + "=0.0";
    echo_to_file(cmd);
}
void StudicaInternal::echo_to_file(std::string st) {
    if (!robot->on_real_robot) {
        return;
    }

    std::ofstream f("/dev/pi-blaster", std::ios::out | std::ios::app);
    if (f.is_open()) {
        f << st << std::endl;
        f.close();
    } else {
        // std::cerr << "Error: Could not write to /dev/pi-blaster" << std::endl;
    }
}
