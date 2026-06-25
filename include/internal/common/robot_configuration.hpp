#pragma once

#include <stdint.h>
#include <string>

class RobotConfiguration
{
public:
    uint8_t camera_index = 0;
    std::string lib_holder_first_path = "/home/pi";
    bool with_pi_blaster = true;
    std::string lidar_port = "/dev/ttyUSB0";

    std::string sim_log_path = "./robocad.log";
    std::string real_log_path = "/var/tmp/robocad.log";
};

class DefaultStudicaConfiguration : public RobotConfiguration
{
public:
    std::string titan_port = "/dev/ttyACM0";
    uint32_t titan_baud = 115200;
    std::string vmx_port = "/dev/spidev1.2";
    uint8_t vmx_ch = 2;
    uint32_t vmx_speed = 1000000;
    uint8_t vmx_mode = 0;
};

class DefaultAlgaritmConfiguration : public RobotConfiguration
{
public:
    DefaultAlgaritmConfiguration() {
        camera_index = 2;
        with_pi_blaster = false;
    }

    std::string titan_port = "/dev/ttyACM0";
    uint32_t titan_baud = 115200;
    std::string vmx_port = "/dev/spidev0.0";
    uint8_t vmx_ch = 0;
    uint32_t vmx_speed = 1000000;
    uint8_t vmx_mode = 0;
};

class DefaultCommonConfiguration : public RobotConfiguration
{
public:
    DefaultCommonConfiguration() {}
};