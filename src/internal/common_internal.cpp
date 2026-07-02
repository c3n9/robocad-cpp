#include "common_internal.hpp"

#include "common/connection_sim.hpp"

#include <chrono>
#include <vector>
#include <thread>
#include <atomic>
#include <cstring>
#include <stdexcept>

// Port of robocad-py common_internal.py. Simulator-only.

#pragma pack(push, 1)
// TX: 22 float (88 bytes) -> matches Python struct.pack('22f')
struct CommonTxPacket {
    float speeds[8];   //  0..7
    float servo[10];   //  8..17
    float leds[4];     // 18..21 (1/0)
};

// RX: <8i4f8Hf8B (76 bytes) -> matches Python struct.unpack
struct CommonRxPacket {
    int32_t enc[8];       // 0..7
    float ultrasound[4];  // 8..11
    uint16_t analog[8];   // 12..19
    float yaw;            // 20
    uint8_t buttons[8];   // 21..28
};
#pragma pack(pop)

static_assert(sizeof(CommonTxPacket) == 88, "Common TX packet must be 88 bytes");
static_assert(sizeof(CommonRxPacket) == 76, "Common RX packet must be 76 bytes");

class RobocadConnectionCommon {
private:
    std::thread update_thread;
    std::atomic<bool> stop_thread{false};
    std::atomic<bool> stopped{false};
    CommonRobotInternal* robot_internal;
    Robot* robot;
    ConnectionSim* connection;

    // Build the 88-byte TX packet from the latest (atomic) robot state.
    std::vector<uint8_t> build_tx() {
        CommonTxPacket tx{};
        tx.speeds[0] = robot_internal->speed_motor_0;
        tx.speeds[1] = robot_internal->speed_motor_1;
        tx.speeds[2] = robot_internal->speed_motor_2;
        tx.speeds[3] = robot_internal->speed_motor_3;
        tx.speeds[4] = robot_internal->speed_motor_4;
        tx.speeds[5] = robot_internal->speed_motor_5;
        tx.speeds[6] = robot_internal->speed_motor_6;
        tx.speeds[7] = robot_internal->speed_motor_7;
        for (int i = 0; i < 10; i++) tx.servo[i] = robot_internal->servo_values[i];
        tx.leds[0] = robot_internal->led_0 ? 1.0f : 0.0f;
        tx.leds[1] = robot_internal->led_1 ? 1.0f : 0.0f;
        tx.leds[2] = robot_internal->led_2 ? 1.0f : 0.0f;
        tx.leds[3] = robot_internal->led_3 ? 1.0f : 0.0f;
        return std::vector<uint8_t>(
            reinterpret_cast<uint8_t*>(&tx),
            reinterpret_cast<uint8_t*>(&tx) + sizeof(tx));
    }

public:
    ~RobocadConnectionCommon() { stop(); }

    void start(ConnectionSim* conn, Robot* r, CommonRobotInternal* i) {
        this->connection = conn;
        this->robot = r;
        this->robot_internal = i;

        robot->power = 12.0; // todo: control from ConnectionSim from robocad

        stop_thread = false;
        update_thread = std::thread(&RobocadConnectionCommon::update_loop, this);
    }

    void stop() {
        if (stopped.exchange(true)) return;
        stop_thread = true;
        if (update_thread.joinable()) update_thread.join();

        // The motor command set just before shutdown may still be sitting in the
        // async TX pipeline (update loop -> talk channel, ~4 ms each). Push the
        // latest state once more and give the talk channel a moment to transmit
        // it before it is torn down, so a "set speed then return" is not lost.
        if (connection) {
            connection->set_data(build_tx());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    void update_loop() {
        while (!stop_thread) {
            // SET DATA
            connection->set_data(build_tx());

            // GET DATA
            auto raw_data = connection->get_data();
            if (raw_data.size() >= sizeof(CommonRxPacket)) {
                CommonRxPacket rx{};
                std::memcpy(&rx, raw_data.data(), sizeof(rx));

                robot_internal->enc_motor_0 = rx.enc[0];
                robot_internal->enc_motor_1 = rx.enc[1];
                robot_internal->enc_motor_2 = rx.enc[2];
                robot_internal->enc_motor_3 = rx.enc[3];
                robot_internal->enc_motor_4 = rx.enc[4];
                robot_internal->enc_motor_5 = rx.enc[5];
                robot_internal->enc_motor_6 = rx.enc[6];
                robot_internal->enc_motor_7 = rx.enc[7];
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

                robot_internal->button_0 = rx.buttons[0] == 1;
                robot_internal->button_1 = rx.buttons[1] == 1;
                robot_internal->button_2 = rx.buttons[2] == 1;
                robot_internal->button_3 = rx.buttons[3] == 1;
                robot_internal->button_4 = rx.buttons[4] == 1;
                robot_internal->button_5 = rx.buttons[5] == 1;
                robot_internal->button_6 = rx.buttons[6] == 1;
                robot_internal->button_7 = rx.buttons[7] == 1;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
    }
};

// ------------- CommonRobotInternal -------------

CommonRobotInternal::CommonRobotInternal(Robot* robot, RobotConfiguration* conf) : robot(robot)
{
    connection = nullptr;
    robocad_connection = nullptr;

    if (robot->on_real_robot)
    {
        throw std::runtime_error("CommonRobot could only be used in simulator");
    }

    connection = new ConnectionSim(robot);
    robocad_connection = new RobocadConnectionCommon();
    robocad_connection->start((ConnectionSim*)connection, robot, this);
}

CommonRobotInternal::~CommonRobotInternal()
{
    stop();
    delete robocad_connection;
    robocad_connection = NULL;
    delete connection;
    connection = NULL;
}

void CommonRobotInternal::stop()
{
    if (robocad_connection) robocad_connection->stop();
    if (connection) connection->stop();
}

cv::Mat CommonRobotInternal::get_camera()
{
    return connection->get_camera();
}

void CommonRobotInternal::set_servo_angle(float angle, int pin)
{
    double dut = 0.000666 * angle + 0.05;
    servo_values[pin] = (float)dut;
}

void CommonRobotInternal::set_servo_pwm(float pwm, int pin)
{
    servo_values[pin] = pwm;
}

void CommonRobotInternal::disable_servo(int pin)
{
    servo_values[pin] = 0.0f;
}
