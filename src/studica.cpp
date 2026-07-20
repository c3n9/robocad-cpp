#include "studica.hpp"

#include <csignal>
#include <stdio.h>

static RobotVmxTitan* current_instance = nullptr;
static void handler(int signum)
{
    current_instance->write_log("Program stopped from handler");
    current_instance->write_log("Signal handler called with signal " + std::to_string(signum));
    current_instance->stop();
    exit(signum);
}

RobotVmxTitan::RobotVmxTitan(bool is_real_robot, DefaultStudicaConfiguration* conf) 
    : Robot(is_real_robot, createDefaultConfIfNull(conf)) 
{
    studica_internal = new StudicaInternal(this, conf_internal);
    reseted_yaw_val = 0;

    // Set the global instance pointer for signal handling
    current_instance = this;

    signal(SIGINT, handler);
    signal(SIGTERM, handler);
}

RobotVmxTitan::~RobotVmxTitan()
{
    delete studica_internal;
    studica_internal = NULL;
    delete conf_internal;
    conf_internal = NULL;
}

void RobotVmxTitan::stop() 
{
    studica_internal->stop();
    write_log("Program stopped.");
}

void RobotVmxTitan::set_motor_speed_0(float speed)
{
    studica_internal->speed_motor_0 = speed;
}
void RobotVmxTitan::set_motor_speed_1(float speed)
{
    studica_internal->speed_motor_1 = speed;
}
void RobotVmxTitan::set_motor_speed_2(float speed)
{
    studica_internal->speed_motor_2 = speed;
}
void RobotVmxTitan::set_motor_speed_3(float speed)
{
    studica_internal->speed_motor_3 = speed;
}

int32_t RobotVmxTitan::get_motor_enc_0()
{
    return studica_internal->enc_motor_0;
}
int32_t RobotVmxTitan::get_motor_enc_1()
{
    return studica_internal->enc_motor_1;
}
int32_t RobotVmxTitan::get_motor_enc_2()
{
    return studica_internal->enc_motor_2;
}
int32_t RobotVmxTitan::get_motor_enc_3()
{
    return studica_internal->enc_motor_3;
}

float RobotVmxTitan::get_yaw() 
{ 
    return rerangeAngle180(studica_internal->yaw - reseted_yaw_val); 
}
void RobotVmxTitan::reset_yaw() 
{ 
    reseted_yaw_val = get_yaw(); 
}

float RobotVmxTitan::get_us1()
{
    return studica_internal->ultrasound_1;
}
float RobotVmxTitan::get_us2()
{
    return studica_internal->ultrasound_2;
}
float RobotVmxTitan::get_analog_1()
{
    return studica_internal->analog_1;
}
float RobotVmxTitan::get_analog_2()
{
    return studica_internal->analog_2;
}
float RobotVmxTitan::get_analog_3()
{
    return studica_internal->analog_3;
}
float RobotVmxTitan::get_analog_4()
{
    return studica_internal->analog_4;
}

std::vector<bool> RobotVmxTitan::get_titan_limits()
{
    return {
        studica_internal->limit_h_0, studica_internal->limit_l_0,
        studica_internal->limit_h_1, studica_internal->limit_l_1,
        studica_internal->limit_h_2, studica_internal->limit_l_2,
        studica_internal->limit_h_3, studica_internal->limit_l_3
    };
}
std::vector<bool> RobotVmxTitan::get_vmx_flex()
{
    return {
        studica_internal->flex_0, studica_internal->flex_1, 
        studica_internal->flex_2, studica_internal->flex_3,
        studica_internal->flex_4, studica_internal->flex_5, 
        studica_internal->flex_6, studica_internal->flex_7
    };
}

cv::Mat RobotVmxTitan::get_camera() 
{
    return studica_internal->get_camera();
}

std::vector<float> RobotVmxTitan::get_lidar() 
{
    return studica_internal->get_lidar();
}

// port is from 1 to 10 included
void RobotVmxTitan::set_servo_angle(float angle, int pin)
{
    studica_internal->set_servo_angle(angle, pin - 1);
}
// port is from 1 to 10 included
void RobotVmxTitan::set_led_state(bool state, int pin)
{
    studica_internal->set_led_state(state, pin - 1);
}
// port is from 1 to 10 included
void RobotVmxTitan::set_servo_pwm(float pwm, int pin)
{
    studica_internal->set_servo_pwm(pwm, pin - 1);
}