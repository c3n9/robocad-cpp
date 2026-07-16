#include "algaritm_internal.hpp"

#include "common/connection_real.hpp"
#include "common/connection_sim.hpp"

#include <chrono>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <algorithm>


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
// Struct for Robocad TX: 28 float (112 bytes) -> matches Python struct.pack('28f')
struct RobocadAlgTxPacket {
    float speeds[4];          //  0..3  motor speeds
    float servo_angles[8];    //  4..11 servo angles
    float additional_servo_1; // 12
    float additional_servo_2; // 13
    float step1_steps;        // 14
    float step2_steps;        // 15
    float step1_steps_per_s;  // 16
    float step2_steps_per_s;  // 17
    float step1_direction;    // 18 (1/0)
    float step2_direction;    // 19 (1/0)
    float use_pid;            // 20 (1/0)
    float p_pid;              // 21
    float i_pid;              // 22
    float d_pid;              // 23
    float outputs[4];         // 24..27 (1/0)
};

// Struct for Robocad RX: <4i4f8H3f14B (74 bytes) -> matches Python struct.unpack
struct RobocadAlgRxPacket {
    int32_t encoders[4];      // 0..3
    float ultrasound[4];      // 4..7
    uint16_t analog[8];       // 8..15
    float yaw;                // 16
    float pitch;              // 17
    float roll;               // 18
    // 14 bytes: 8 limits, 4 inputs, 2 step-busy
    uint8_t flags[14];        // 19..32
};
#pragma pack(pop)

static_assert(sizeof(RobocadAlgTxPacket) == 112, "Algaritm TX packet must be 112 bytes");
static_assert(sizeof(RobocadAlgRxPacket) == 74, "Algaritm RX packet must be 74 bytes");

class RobocadConnectionAlgaritm {
private:
    std::thread update_thread;
    std::atomic<bool> stop_thread{false};
    std::atomic<bool> stopped{false};
    AlgaritmInternal* robot_internal;
    Robot* robot;
    ConnectionSim* connection;

    // Build the 112-byte TX packet from the latest (atomic) robot state.
    std::vector<uint8_t> build_tx() {
        RobocadAlgTxPacket tx{};
        tx.speeds[0] = robot_internal->speed_motor_0;
        tx.speeds[1] = robot_internal->speed_motor_1;
        tx.speeds[2] = robot_internal->speed_motor_2;
        tx.speeds[3] = robot_internal->speed_motor_3;
        for (int i = 0; i < 8; i++) tx.servo_angles[i] = robot_internal->servo_angles[i] * 0.0011111f + 0.05f;
        tx.additional_servo_1 = robot_internal->additional_servo_1;
        tx.additional_servo_2 = robot_internal->additional_servo_2;
        tx.step1_steps = static_cast<float>(robot_internal->step_motor_1_steps);
        tx.step2_steps = static_cast<float>(robot_internal->step_motor_2_steps);
        tx.step1_steps_per_s = static_cast<float>(robot_internal->step_motor_1_steps_per_s);
        tx.step2_steps_per_s = static_cast<float>(robot_internal->step_motor_2_steps_per_s);
        tx.step1_direction = robot_internal->step_motor_1_direction ? 1.0f : 0.0f;
        tx.step2_direction = robot_internal->step_motor_2_direction ? 1.0f : 0.0f;
        tx.use_pid = robot_internal->use_pid ? 1.0f : 0.0f;
        tx.p_pid = robot_internal->p_pid;
        tx.i_pid = robot_internal->i_pid;
        tx.d_pid = robot_internal->d_pid;
        for (int i = 0; i < 4; i++) tx.outputs[i] = robot_internal->outputs[i] ? 1.0f : 0.0f;
        return std::vector<uint8_t>(
            reinterpret_cast<uint8_t*>(&tx),
            reinterpret_cast<uint8_t*>(&tx) + sizeof(tx));
    }

public:
    ~RobocadConnectionAlgaritm() { stop(); }

    void start(ConnectionSim* conn, Robot* r, AlgaritmInternal* i) {
        this->connection = conn;
        this->robot = r;
        this->robot_internal = i;

        robot->power = 12.0; // todo: control from ConnectionSim from robocad

        stop_thread = false;
        update_thread = std::thread(&RobocadConnectionAlgaritm::update_loop, this);
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
            if (raw_data.size() >= sizeof(RobocadAlgRxPacket)) {
                RobocadAlgRxPacket rx{};
                std::memcpy(&rx, raw_data.data(), sizeof(rx));

                robot_internal->enc_motor_0 = rx.encoders[0];
                robot_internal->enc_motor_1 = rx.encoders[1];
                robot_internal->enc_motor_2 = rx.encoders[2];
                robot_internal->enc_motor_3 = rx.encoders[3];
                robot_internal->ultrasound_1 = rx.ultrasound[0];
                robot_internal->ultrasound_2 = rx.ultrasound[1];
                robot_internal->ultrasound_3 = rx.ultrasound[2];
                robot_internal->ultrasound_4 = rx.ultrasound[3];
                robot_internal->analog_1 = rx.analog[0];
                robot_internal->analog_2 = rx.analog[1];
                robot_internal->analog_3 = rx.analog[2];
                robot_internal->analog_4 = rx.analog[3];
                robot_internal->analog_5 = rx.analog[4];
                robot_internal->analog_6 = rx.analog[5];
                robot_internal->analog_7 = rx.analog[6];
                robot_internal->analog_8 = rx.analog[7];
                robot_internal->yaw = rx.yaw;
                robot_internal->pitch = rx.pitch;
                robot_internal->roll = rx.roll;

                robot_internal->limit_h_0 = rx.flags[0] == 1;
                robot_internal->limit_l_0 = rx.flags[1] == 1;
                robot_internal->limit_h_1 = rx.flags[2] == 1;
                robot_internal->limit_l_1 = rx.flags[3] == 1;
                robot_internal->limit_h_2 = rx.flags[4] == 1;
                robot_internal->limit_l_2 = rx.flags[5] == 1;
                robot_internal->limit_h_3 = rx.flags[6] == 1;
                robot_internal->limit_l_3 = rx.flags[7] == 1;

                robot_internal->inputs[0] = rx.flags[8] == 1;
                robot_internal->inputs[1] = rx.flags[9] == 1;
                robot_internal->inputs[2] = rx.flags[10] == 1;
                robot_internal->inputs[3] = rx.flags[11] == 1;

                robot_internal->is_step_1_busy = rx.flags[12] == 1;
                robot_internal->is_step_2_busy = rx.flags[13] == 1;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
    }
};

// --- TitanCOM ---

class TitanCOMAlgaritm {
private:
    std::thread th;
    std::atomic<bool> stop_th{false};
    ConnectionReal* connection;
    Robot* robot;
    AlgaritmInternal* robot_internal;
    DefaultAlgaritmConfiguration* conf;

public:
    TitanCOMAlgaritm() : connection(nullptr), robot(nullptr), robot_internal(nullptr), conf(nullptr) {}
    ~TitanCOMAlgaritm() { stop(); }

    void start_com(ConnectionReal* conn, Robot* rb, AlgaritmInternal* internal, DefaultAlgaritmConfiguration* cfg) {
        connection = conn;
        robot = rb;
        robot_internal = internal;
        conf = cfg;
        stop_th = false;
        th = std::thread(&TitanCOMAlgaritm::com_loop, this);
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
        if (data.size() < 41) return;
        if (data[0] == 1) {
            if (data[40] == 222) {
                robot_internal->enc_motor_0 = (data[4] << 24) | (data[3] << 16) | (data[2] << 8) | data[1];
                robot_internal->enc_motor_1 = (data[8] << 24) | (data[7] << 16) | (data[6] << 8) | data[5];
                robot_internal->enc_motor_2 = (data[12] << 24) | (data[11] << 16) | (data[10] << 8) | data[9];
                robot_internal->enc_motor_3 = (data[16] << 24) | (data[15] << 16) | (data[14] << 8) | data[13];

                // bit order matches Python: limit_l_0 = bit 7 ... limit_h_3 = bit 0
                robot_internal->limit_l_0 = access_bit(data[17], 7);
                robot_internal->limit_h_0 = access_bit(data[17], 6);
                robot_internal->limit_l_1 = access_bit(data[17], 5);
                robot_internal->limit_h_1 = access_bit(data[17], 4);
                robot_internal->limit_l_2 = access_bit(data[17], 3);
                robot_internal->limit_h_2 = access_bit(data[17], 2);
                robot_internal->limit_l_3 = access_bit(data[17], 1);
                robot_internal->limit_h_3 = access_bit(data[17], 0);

                robot_internal->is_step_1_busy = (data[18] != 0);
                robot_internal->is_step_2_busy = (data[19] != 0);
            }
        }
        else
        {
            robot->write_log("received wrong data");
        }
    }

    std::vector<uint8_t> set_up_tx_data() {
        std::vector<uint8_t> tx_data(48, 0);
        tx_data[0] = 1;

        auto pack_speed = [](float speed) -> uint8_t {
            int8_t clipped = static_cast<int8_t>(std::clamp(speed, -100.0f, 100.0f));
            return static_cast<uint8_t>(clipped);
        };

        tx_data[1] = pack_speed(robot_internal->speed_motor_0);
        tx_data[2] = pack_speed(robot_internal->speed_motor_1);
        tx_data[3] = pack_speed(robot_internal->speed_motor_2);
        tx_data[4] = pack_speed(robot_internal->speed_motor_3);

        // bits: '11' + step1_dir + step2_dir + use_pid + '001'
        uint8_t dir_byte = 0b11000001;
        if (robot_internal->step_motor_1_direction) dir_byte |= (1 << 5);
        if (robot_internal->step_motor_2_direction) dir_byte |= (1 << 4);
        if (robot_internal->use_pid)                dir_byte |= (1 << 3);
        tx_data[5] = dir_byte;

        tx_data[6] = static_cast<uint8_t>(robot_internal->additional_servo_1);
        tx_data[7] = static_cast<uint8_t>(robot_internal->additional_servo_2);

        auto write_int32_be = [&](int32_t value, size_t index) {
            uint32_t u_val = static_cast<uint32_t>(std::abs(value));
            tx_data[index]     = (u_val >> 24) & 0xFF;
            tx_data[index + 1] = (u_val >> 16) & 0xFF;
            tx_data[index + 2] = (u_val >> 8) & 0xFF;
            tx_data[index + 3] = u_val & 0xFF;
        };

        write_int32_be(robot_internal->step_motor_1_steps, 8);
        write_int32_be(robot_internal->step_motor_2_steps, 12);
        write_int32_be(robot_internal->step_motor_1_steps_per_s, 16);
        write_int32_be(robot_internal->step_motor_2_steps_per_s, 20);

        auto write_float_le = [&](float value, size_t index) {
            std::memcpy(&tx_data[index], &value, sizeof(float));
        };

        write_float_le(robot_internal->p_pid, 24);
        write_float_le(robot_internal->i_pid, 28);
        write_float_le(robot_internal->d_pid, 32);

        tx_data[40] = 222;

        return tx_data;
    }
};

class VMXSPIAlgaritm {
private:
    std::thread th;
    std::atomic<bool> stop_th{false};
    ConnectionReal* connection;
    Robot* robot;
    AlgaritmInternal* robot_internal;
    DefaultAlgaritmConfiguration* conf;
    int toggler = 0;

public:
    VMXSPIAlgaritm() : connection(nullptr), robot(nullptr), robot_internal(nullptr), conf(nullptr) {}
    ~VMXSPIAlgaritm() { stop(); }

    void start_spi(ConnectionReal* conn, Robot* rb, AlgaritmInternal* internal, DefaultAlgaritmConfiguration* cfg) {
        connection = conn;
        robot = rb;
        robot_internal = internal;
        conf = cfg;
        toggler = 0;
        stop_th = false;
        th = std::thread(&VMXSPIAlgaritm::spi_loop, this);
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
            robot_internal->analog_1 = (data[2] << 8) | data[1];
            robot_internal->analog_2 = (data[4] << 8) | data[3];
            robot_internal->analog_3 = (data[6] << 8) | data[5];
            robot_internal->analog_4 = (data[8] << 8) | data[7];
            robot_internal->analog_5 = (data[10] << 8) | data[9];
            robot_internal->analog_6 = (data[12] << 8) | data[11];
            robot_internal->analog_7 = (data[14] << 8) | data[13];
        } else if (data[0] == 2) {
            robot_internal->analog_8 = (data[2] << 8) | data[1];

            int us1_ui = (data[4] << 8) | data[3];
            robot_internal->ultrasound_1 = us1_ui / 100.0;
            int us2_ui = (data[6] << 8) | data[5];
            robot_internal->ultrasound_2 = us2_ui / 100.0;
            int us3_ui = (data[8] << 8) | data[7];
            robot_internal->ultrasound_3 = us3_ui / 100.0;
            int us4_ui = (data[10] << 8) | data[9];
            robot_internal->ultrasound_4 = us4_ui / 100.0;

            robot_internal->inputs[0] = access_bit(data[11], 0);
            robot_internal->inputs[1] = access_bit(data[11], 1);
            robot_internal->inputs[2] = access_bit(data[11], 2);
            robot_internal->inputs[3] = access_bit(data[11], 3);
        } else if (data[0] == 3) {
            int yaw_ui = (data[2] << 8) | data[1];
            double new_yaw = (yaw_ui / 100.0) * (access_bit(data[7], 1) ? 1.0 : -1.0);
            double delta_yaw = calc_angle_unlim(new_yaw, robot_internal->yaw);
            robot_internal->yaw_unlim = robot_internal->yaw_unlim + delta_yaw;
            robot_internal->yaw = new_yaw;

            int pitch_ui = (data[4] << 8) | data[3];
            double new_pitch = (pitch_ui / 100.0) * (access_bit(data[7], 2) ? 1.0 : -1.0);
            double delta_pitch = calc_angle_unlim(new_pitch, robot_internal->pitch);
            robot_internal->pitch_unlim = robot_internal->pitch_unlim + delta_pitch;
            robot_internal->pitch = new_pitch;

            int roll_ui = (data[6] << 8) | data[5];
            double new_roll = (roll_ui / 100.0) * (access_bit(data[7], 3) ? 1.0 : -1.0);
            double delta_roll = calc_angle_unlim(new_roll, robot_internal->roll);
            robot_internal->roll_unlim = robot_internal->roll_unlim + delta_roll;
            robot_internal->roll = new_roll;

            double power = ((data[9] << 8) | data[8]) / 100.0;
            robot->power = power;
        }
    }

    std::vector<uint8_t> set_up_tx_data() {
        std::vector<uint8_t> tx_list(16, 0);
        if (toggler == 0) {
            tx_list[0] = 1;

            tx_list[1] = static_cast<uint8_t>(robot_internal->servo_angles[0]);
            tx_list[2] = static_cast<uint8_t>(robot_internal->servo_angles[1]);
            tx_list[3] = static_cast<uint8_t>(robot_internal->servo_angles[2]);
            tx_list[4] = static_cast<uint8_t>(robot_internal->servo_angles[3]);
            tx_list[5] = static_cast<uint8_t>(robot_internal->servo_angles[4]);
            tx_list[6] = static_cast<uint8_t>(robot_internal->servo_angles[5]);
            tx_list[7] = static_cast<uint8_t>(robot_internal->servo_angles[6]);
            tx_list[8] = static_cast<uint8_t>(robot_internal->servo_angles[7]);

            // bits: '1' + out0 + out1 + out2 + out3 + '001'
            uint8_t out_byte = 0b10000001;
            if (robot_internal->outputs[0]) out_byte |= (1 << 6);
            if (robot_internal->outputs[1]) out_byte |= (1 << 5);
            if (robot_internal->outputs[2]) out_byte |= (1 << 4);
            if (robot_internal->outputs[3]) out_byte |= (1 << 3);
            tx_list[9] = out_byte;
        }
        return tx_list;
    }

    double calc_angle_unlim(double new_angle, double old_angle) {
        double delta_angle = new_angle - old_angle;
        if (delta_angle < -180.0) {
            delta_angle = (180.0 - old_angle) + (180.0 + new_angle);
        } else if (delta_angle > 180.0) {
            delta_angle = -(180.0 + old_angle) - (180.0 - new_angle);
        }
        return delta_angle;
    }
};

// ------------- AlgaritmInternal -------------

AlgaritmInternal::AlgaritmInternal(Robot* robot, RobotConfiguration* conf) : robot(robot) 
{
    titan_com = nullptr;
    vmx_spi = nullptr;
    robocad_connection = nullptr;
    updater = nullptr;

    if (robot->on_real_robot)
    {
        updater = new RepkaUpdater(robot);
        connection = new ConnectionReal(robot, updater, conf);
        titan_com = new TitanCOMAlgaritm();
        titan_com->start_com((ConnectionReal*)connection, robot, this, (DefaultAlgaritmConfiguration*)conf);
        vmx_spi = new VMXSPIAlgaritm();
        vmx_spi->start_spi((ConnectionReal*)connection, robot, this, (DefaultAlgaritmConfiguration*)conf);
    }
    else
    {
        connection = new ConnectionSim(robot);
        robocad_connection = new RobocadConnectionAlgaritm();
        robocad_connection->start((ConnectionSim*)connection, robot, this);
    }
}

AlgaritmInternal::~AlgaritmInternal()
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

void AlgaritmInternal::stop()
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

cv::Mat AlgaritmInternal::get_camera() 
{
    return connection->get_camera();
}

std::vector<float> AlgaritmInternal::get_lidar() 
{
    return connection->get_lidar();
}

void AlgaritmInternal::set_servo_angle(float angle, int pin)
{
    servo_angles[pin] = angle;
}

void AlgaritmInternal::step_motor_move(int num, int steps, int steps_per_second, bool direction)
{
    if (num == 1)
    {
        step_motor_1_steps = steps;
        step_motor_1_steps_per_s = steps_per_second;
        step_motor_1_direction = direction;
    }
    else if (num == 2)
    {
        step_motor_2_steps = steps;
        step_motor_2_steps_per_s = steps_per_second;
        step_motor_2_direction = direction;
    }
}

