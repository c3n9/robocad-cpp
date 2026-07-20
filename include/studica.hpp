#pragma once

#include "common/robot.hpp"
#include "studica_internal.hpp"

#include <opencv2/opencv.hpp>

#include <chrono>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

class RobotVmxTitan : public Robot
{
public:
    RobotVmxTitan(bool is_real_robot = false, DefaultStudicaConfiguration* conf = nullptr);
    ~RobotVmxTitan();

    void stop();
    cv::Mat get_camera();
    std::vector<float> get_lidar();

    void set_motor_speed_0(float speed);
    void set_motor_speed_1(float speed);
    void set_motor_speed_2(float speed);
    void set_motor_speed_3(float speed);

    int32_t get_motor_enc_0();
    int32_t get_motor_enc_1();
    int32_t get_motor_enc_2();
    int32_t get_motor_enc_3();

    float get_yaw();
    void reset_yaw();
    float get_us1();
    float get_us2();
    float get_analog_1();
    float get_analog_2();
    float get_analog_3();
    float get_analog_4();

    std::vector<bool> get_titan_limits();
    std::vector<bool> get_vmx_flex();

    // port is from 1 to 10 included
    void set_servo_angle(float angle, int pin);
    // port is from 1 to 10 included
    void set_led_state(bool state, int pin);
    // port is from 1 to 10 included
    void set_servo_pwm(float pwm, int pin);

private:
    DefaultStudicaConfiguration* conf_internal;
    StudicaInternal* studica_internal;
    float reseted_yaw_val;

    DefaultStudicaConfiguration* createDefaultConfIfNull(DefaultStudicaConfiguration* conf)
    {
        if (conf == nullptr) {
            DefaultStudicaConfiguration* default_conf = new DefaultStudicaConfiguration();
            conf_internal = default_conf;
            return default_conf;
        }
        conf_internal = conf;
        return conf;
    }

    float rerangeAngle180(float angle)
    {
        while (angle > 180)
            angle -= 360;
        while (angle < -180)
            angle += 360;
        return angle;
    }
};