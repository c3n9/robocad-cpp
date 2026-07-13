#include "algaritm.hpp"

#include <csignal>
#include <stdio.h>

static RobotAlgaritm* current_instance = nullptr;
static void handler(int signum)
{
    current_instance->write_log("Program stopped from handler");
    current_instance->write_log("Signal handler called with signal " + std::to_string(signum));
    current_instance->stop();
    exit(signum);
}

RobotAlgaritm::RobotAlgaritm(bool is_real_robot, DefaultAlgaritmConfiguration* conf) 
    : Robot(is_real_robot, createDefaultConfIfNull(conf)) 
{
    algaritm_internal = new AlgaritmInternal(this, conf_internal);

    // expose the internal digital outputs array directly (robot.outputs[pin] = value)
    outputs = algaritm_internal->outputs;

    // Set the global instance pointer for signal handling
    current_instance = this;
    
    signal(SIGINT, handler);
    signal(SIGTERM, handler);
}

RobotAlgaritm::~RobotAlgaritm()
{
    delete algaritm_internal;
    algaritm_internal = NULL;
    delete conf_internal;
    conf_internal = NULL;
}

void RobotAlgaritm::stop() 
{
    algaritm_internal->stop();
    write_log("Program stopped.");
}

void RobotAlgaritm::set_motor_speed_0(float speed)
{
    algaritm_internal->speed_motor_0 = speed;
}
void RobotAlgaritm::set_motor_speed_1(float speed)
{
    algaritm_internal->speed_motor_1 = speed;
}
void RobotAlgaritm::set_motor_speed_2(float speed)
{
    algaritm_internal->speed_motor_2 = speed;
}
void RobotAlgaritm::set_motor_speed_3(float speed)
{
    algaritm_internal->speed_motor_3 = speed;
}

int32_t RobotAlgaritm::get_motor_enc_0()
{
    return algaritm_internal->enc_motor_0;
}
int32_t RobotAlgaritm::get_motor_enc_1()
{
    return algaritm_internal->enc_motor_1;
}
int32_t RobotAlgaritm::get_motor_enc_2()
{
    return algaritm_internal->enc_motor_2;
}
int32_t RobotAlgaritm::get_motor_enc_3()
{
    return algaritm_internal->enc_motor_3;
}

float RobotAlgaritm::get_yaw()
{
    return algaritm_internal->yaw;
}
float RobotAlgaritm::get_pitch()
{
    return algaritm_internal->pitch;
}
float RobotAlgaritm::get_roll()
{
    return algaritm_internal->roll;
}
float RobotAlgaritm::get_us1()
{
    return algaritm_internal->ultrasound_1;
}
float RobotAlgaritm::get_us2()
{
    return algaritm_internal->ultrasound_2;
}
float RobotAlgaritm::get_us3()
{
    return algaritm_internal->ultrasound_3;
}
float RobotAlgaritm::get_us4()
{
    return algaritm_internal->ultrasound_4;
}
float RobotAlgaritm::get_analog_1()
{
    return algaritm_internal->analog_1;
}
float RobotAlgaritm::get_analog_2()
{
    return algaritm_internal->analog_2;
}
float RobotAlgaritm::get_analog_3()
{
    return algaritm_internal->analog_3;
}
float RobotAlgaritm::get_analog_4()
{
    return algaritm_internal->analog_4;
}
float RobotAlgaritm::get_analog_5()
{
    return algaritm_internal->analog_5;
}
float RobotAlgaritm::get_analog_6()
{
    return algaritm_internal->analog_6;
}
float RobotAlgaritm::get_analog_7()
{
    return algaritm_internal->analog_7;
}
float RobotAlgaritm::get_analog_8()
{
    return algaritm_internal->analog_8;
}

void RobotAlgaritm::set_additional_servo_1(float angle)
{
    algaritm_internal->additional_servo_1 = angle;
}
void RobotAlgaritm::set_additional_servo_2(float angle)
{
    algaritm_internal->additional_servo_2 = angle;
}
void RobotAlgaritm::set_pid_settings(bool use_pid, float p, float i, float d)
{
    algaritm_internal->use_pid = use_pid;
    algaritm_internal->p_pid = p;
    algaritm_internal->i_pid = i;
    algaritm_internal->d_pid = d;
}
void RobotAlgaritm::step_motor_move(int num, int steps, int steps_per_second, bool direction)
{
    algaritm_internal->step_motor_move(num, steps, steps_per_second, direction);
}
bool RobotAlgaritm::is_step_1_busy()
{
    return algaritm_internal->is_step_1_busy;
}
bool RobotAlgaritm::is_step_2_busy()
{
    return algaritm_internal->is_step_2_busy;
}

std::vector<bool> RobotAlgaritm::get_titan_limits()
{
    return {
        algaritm_internal->limit_h_0, algaritm_internal->limit_l_0,
        algaritm_internal->limit_h_1, algaritm_internal->limit_l_1,
        algaritm_internal->limit_h_2, algaritm_internal->limit_l_2,
        algaritm_internal->limit_h_3, algaritm_internal->limit_l_3
    };
}

std::vector<bool> RobotAlgaritm::get_inputs()
{
    return {
        algaritm_internal->inputs[0], algaritm_internal->inputs[1],
        algaritm_internal->inputs[2], algaritm_internal->inputs[3]
    };
}

cv::Mat RobotAlgaritm::get_camera()
{
    return algaritm_internal->get_camera();
}

std::vector<float> RobotAlgaritm::get_lidar() 
{
    return algaritm_internal->get_lidar();
}

// port is from 1 to 8 included
void RobotAlgaritm::set_servo_angle(float angle, int pin)
{
    algaritm_internal->set_servo_angle(angle, pin - 1);
}