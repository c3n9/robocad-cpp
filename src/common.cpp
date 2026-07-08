#include "common.hpp"

#include <csignal>
#include <stdio.h>

static CommonRobot* current_instance = nullptr;
static void handler(int signum)
{
    current_instance->write_log("Program stopped from handler");
    current_instance->write_log("Signal handler called with signal " + std::to_string(signum));
    current_instance->stop();
    exit(signum);
}

CommonRobot::CommonRobot(bool is_real_robot, DefaultCommonConfiguration* conf)
    : Robot(is_real_robot, createDefaultConfIfNull(conf))
{
    common_internal = new CommonRobotInternal(this, conf_internal);

    current_instance = this;
    signal(SIGINT, handler);
    signal(SIGTERM, handler);
}

CommonRobot::~CommonRobot()
{
    delete common_internal;
    common_internal = NULL;
    delete conf_internal;
    conf_internal = NULL;
}

void CommonRobot::stop()
{
    common_internal->stop();
    write_log("Program stopped.");
}

void CommonRobot::set_motor_speed_0(float speed) { common_internal->speed_motor_0 = speed; }
void CommonRobot::set_motor_speed_1(float speed) { common_internal->speed_motor_1 = speed; }
void CommonRobot::set_motor_speed_2(float speed) { common_internal->speed_motor_2 = speed; }
void CommonRobot::set_motor_speed_3(float speed) { common_internal->speed_motor_3 = speed; }
void CommonRobot::set_motor_speed_4(float speed) { common_internal->speed_motor_4 = speed; }
void CommonRobot::set_motor_speed_5(float speed) { common_internal->speed_motor_5 = speed; }
void CommonRobot::set_motor_speed_6(float speed) { common_internal->speed_motor_6 = speed; }
void CommonRobot::set_motor_speed_7(float speed) { common_internal->speed_motor_7 = speed; }

int32_t CommonRobot::get_motor_enc_0() { return common_internal->enc_motor_0; }
int32_t CommonRobot::get_motor_enc_1() { return common_internal->enc_motor_1; }
int32_t CommonRobot::get_motor_enc_2() { return common_internal->enc_motor_2; }
int32_t CommonRobot::get_motor_enc_3() { return common_internal->enc_motor_3; }
int32_t CommonRobot::get_motor_enc_4() { return common_internal->enc_motor_4; }
int32_t CommonRobot::get_motor_enc_5() { return common_internal->enc_motor_5; }
int32_t CommonRobot::get_motor_enc_6() { return common_internal->enc_motor_6; }
int32_t CommonRobot::get_motor_enc_7() { return common_internal->enc_motor_7; }

float CommonRobot::get_yaw() { return common_internal->yaw; }
float CommonRobot::get_us1() { return common_internal->ultrasound_1; }
float CommonRobot::get_us2() { return common_internal->ultrasound_2; }
float CommonRobot::get_us3() { return common_internal->ultrasound_3; }
float CommonRobot::get_us4() { return common_internal->ultrasound_4; }
float CommonRobot::get_analog_1() { return common_internal->analog_1; }
float CommonRobot::get_analog_2() { return common_internal->analog_2; }
float CommonRobot::get_analog_3() { return common_internal->analog_3; }
float CommonRobot::get_analog_4() { return common_internal->analog_4; }
float CommonRobot::get_analog_5() { return common_internal->analog_5; }
float CommonRobot::get_analog_6() { return common_internal->analog_6; }
float CommonRobot::get_analog_7() { return common_internal->analog_7; }
float CommonRobot::get_analog_8() { return common_internal->analog_8; }

std::vector<bool> CommonRobot::get_buttons()
{
    return {
        common_internal->button_0, common_internal->button_1,
        common_internal->button_2, common_internal->button_3,
        common_internal->button_4, common_internal->button_5,
        common_internal->button_6, common_internal->button_7
    };
}

void CommonRobot::set_led_0(bool value) { common_internal->led_0 = value; }
void CommonRobot::set_led_1(bool value) { common_internal->led_1 = value; }
void CommonRobot::set_led_2(bool value) { common_internal->led_2 = value; }
void CommonRobot::set_led_3(bool value) { common_internal->led_3 = value; }

cv::Mat CommonRobot::get_camera()
{
    return common_internal->get_camera();
}

// port is from 1 to 10 included
void CommonRobot::set_servo_angle(float value, int port)
{
    common_internal->set_servo_angle(value, port - 1);
}
// port is from 1 to 10 included
void CommonRobot::set_servo_pwm(float value, int port)
{
    common_internal->set_servo_pwm(value, port - 1);
}
