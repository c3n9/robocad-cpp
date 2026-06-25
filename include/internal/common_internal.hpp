#pragma once

#include "common/connection_base.hpp"
#include "common/robot.hpp"
#include "common/robot_configuration.hpp"

#include <opencv2/opencv.hpp>
#include <atomic>

class RobocadConnectionCommon;

// Port of robocad-py CommonRobotInternal (common_internal.py). Simulator-only.
class CommonRobotInternal
{
public:
    std::atomic<float> speed_motor_0 = 0.0f, speed_motor_1 = 0.0f, speed_motor_2 = 0.0f, speed_motor_3 = 0.0f;
    std::atomic<float> speed_motor_4 = 0.0f, speed_motor_5 = 0.0f, speed_motor_6 = 0.0f, speed_motor_7 = 0.0f;
    std::atomic<int32_t> enc_motor_0 = 0, enc_motor_1 = 0, enc_motor_2 = 0, enc_motor_3 = 0;
    std::atomic<int32_t> enc_motor_4 = 0, enc_motor_5 = 0, enc_motor_6 = 0, enc_motor_7 = 0;
    std::atomic<bool> button_0 = false, button_1 = false, button_2 = false, button_3 = false;
    std::atomic<bool> button_4 = false, button_5 = false, button_6 = false, button_7 = false;

    std::atomic<float> yaw = 0.0f;
    std::atomic<float> ultrasound_1 = 0.0f, ultrasound_2 = 0.0f, ultrasound_3 = 0.0f, ultrasound_4 = 0.0f;
    std::atomic<int32_t> analog_1 = 0, analog_2 = 0, analog_3 = 0, analog_4 = 0;
    std::atomic<int32_t> analog_5 = 0, analog_6 = 0, analog_7 = 0, analog_8 = 0;
    std::atomic<bool> led_0 = false, led_1 = false, led_2 = false, led_3 = false;
    std::atomic<float> servo_values[10] = {0.0f};

    CommonRobotInternal(Robot* robot, RobotConfiguration* conf);
    ~CommonRobotInternal();

    void stop();
    cv::Mat get_camera();

    void set_servo_angle(float angle, int pin);
    void set_servo_pwm(float pwm, int pin);
    void disable_servo(int pin);

private:
    Robot* robot;
    ConnectionBase* connection;
    RobocadConnectionCommon* robocad_connection;
};
