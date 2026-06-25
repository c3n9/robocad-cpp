#pragma once

#include "common/robot.hpp"
#include "common_internal.hpp"

#include <opencv2/opencv.hpp>

#include <vector>

// Port of robocad-py CommonRobot (common.py). Simulator-only.
class CommonRobot : public Robot
{
public:
    CommonRobot(bool is_real_robot = false, DefaultCommonConfiguration* conf = nullptr);
    ~CommonRobot();

    void stop();
    cv::Mat get_camera();

    void set_motor_speed_0(float speed);
    void set_motor_speed_1(float speed);
    void set_motor_speed_2(float speed);
    void set_motor_speed_3(float speed);
    void set_motor_speed_4(float speed);
    void set_motor_speed_5(float speed);
    void set_motor_speed_6(float speed);
    void set_motor_speed_7(float speed);

    int32_t get_motor_enc_0();
    int32_t get_motor_enc_1();
    int32_t get_motor_enc_2();
    int32_t get_motor_enc_3();
    int32_t get_motor_enc_4();
    int32_t get_motor_enc_5();
    int32_t get_motor_enc_6();
    int32_t get_motor_enc_7();

    float get_yaw();
    float get_us1();
    float get_us2();
    float get_us3();
    float get_us4();
    float get_analog_1();
    float get_analog_2();
    float get_analog_3();
    float get_analog_4();
    float get_analog_5();
    float get_analog_6();
    float get_analog_7();
    float get_analog_8();

    std::vector<bool> get_buttons();

    void set_led_0(bool value);
    void set_led_1(bool value);
    void set_led_2(bool value);
    void set_led_3(bool value);

    // port is from 1 to 10 included
    void set_servo_angle(float value, int port);
    // port is from 1 to 10 included
    void set_servo_pwm(float value, int port);

private:
    DefaultCommonConfiguration* conf_internal;
    CommonRobotInternal* common_internal;

    DefaultCommonConfiguration* createDefaultConfIfNull(DefaultCommonConfiguration* conf)
    {
        if (conf == nullptr) {
            DefaultCommonConfiguration* default_conf = new DefaultCommonConfiguration();
            conf_internal = default_conf;
            return default_conf;
        }
        conf_internal = conf;
        return conf;
    }
};
