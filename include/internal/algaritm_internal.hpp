#pragma once

#include "common/connection_base.hpp"
#include "common/robot.hpp"
#include "common/updaters.hpp"
#include "common/shared.hpp"
#include "common/robot_configuration.hpp"

class RobocadConnectionAlgaritm;
class TitanCOMAlgaritm;
class VMXSPIAlgaritm;

class AlgaritmInternal 
{
public:
    std::atomic<float> speed_motor_0 = 0.0f, speed_motor_1 = 0.0f, speed_motor_2 = 0.0f, speed_motor_3 = 0.0f;
    std::atomic<int32_t> enc_motor_0 = 0, enc_motor_1 = 0, enc_motor_2 = 0, enc_motor_3 = 0;
    std::atomic<float> additional_servo_1 = 0.0, additional_servo_2 = 0.0;
    std::atomic<bool> is_step_1_busy = false, is_step_2_busy = false;
    std::atomic<int32_t> step_motor_1_steps = 0, step_motor_2_steps = 0;
    std::atomic<int32_t> step_motor_1_steps_per_s = 0, step_motor_2_steps_per_s = 0;
    std::atomic<bool> step_motor_1_direction = false, step_motor_2_direction = false;
    std::atomic<bool> use_pid = false;
    std::atomic<float> p_pid = 0.14f, i_pid = 0.1f, d_pid = 0.0f;

    std::atomic<float> yaw = 0.0f, yaw_unlim = 0.0f;
    std::atomic<float> pitch = 0.0f, pitch_unlim = 0.0f;
    std::atomic<float> roll = 0.0f, roll_unlim = 0.0f;
    std::atomic<float> ultrasound_1 = 0.0f, ultrasound_2 = 0.0f, ultrasound_3 = 0.0f, ultrasound_4 = 0.0f;
    std::atomic<int32_t> analog_1 = 0, analog_2 = 0, analog_3 = 0, analog_4 = 0, analog_5 = 0, analog_6 = 0, analog_7 = 0, analog_8 = 0;
    std::atomic<float> servo_angles[8] = {0.0f};

    std::atomic<bool> limit_l_0 = false, limit_h_0 = false, limit_l_1 = false, limit_h_1 = false, limit_l_2 = false, limit_h_2 = false, limit_l_3 = false, limit_h_3 = false;

    std::atomic<bool> inputs[4] = {false, false, false, false};
    std::atomic<bool> outputs[4] = {false, false, false, false};

    AlgaritmInternal(Robot* robot, RobotConfiguration* conf);
    ~AlgaritmInternal();

    void stop();
    cv::Mat get_camera();
    std::vector<float> get_lidar();

    void set_servo_angle(float angle, int pin);
    void set_output(int pin, bool value);
    void step_motor_move(int num, int steps, int steps_per_second, bool direction);

private:
    Robot* robot;
    ConnectionBase* connection;
    Updater* updater;

    RobocadConnectionAlgaritm* robocad_connection;
    TitanCOMAlgaritm* titan_com;
    VMXSPIAlgaritm* vmx_spi;
};