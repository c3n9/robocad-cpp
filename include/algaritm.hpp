#pragma once

#include "common/robot.hpp"
#include "algaritm_internal.hpp"

#include <opencv2/opencv.hpp>

#include <chrono>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

class RobotAlgaritm : public Robot
{
public:
    RobotAlgaritm(bool is_real_robot = false, DefaultAlgaritmConfiguration* conf = nullptr);
    ~RobotAlgaritm();

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
    float get_pitch();
    float get_roll();
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

    void set_additional_servo_1(float angle);
    void set_additional_servo_2(float angle);
    void set_pid_settings(bool use_pid, float p, float i, float d);
    // num 1 or 2
    void step_motor_move(int num, int steps, int steps_per_second, bool direction);

    bool is_step_1_busy();
    bool is_step_2_busy();

    std::vector<bool> get_titan_limits();

    std::vector<bool> get_inputs();

    std::atomic<bool>* outputs = nullptr;

    // port is from 1 to 8 included
    void set_servo_angle(float angle, int pin);

private:
    DefaultAlgaritmConfiguration* conf_internal;
    AlgaritmInternal* algaritm_internal;

    DefaultAlgaritmConfiguration* createDefaultConfIfNull(DefaultAlgaritmConfiguration* conf)
    {
        if (conf == nullptr) {
            DefaultAlgaritmConfiguration* default_conf = new DefaultAlgaritmConfiguration();
            conf_internal = default_conf;
            return default_conf;
        }
        conf_internal = conf;
        return conf;
    }
};