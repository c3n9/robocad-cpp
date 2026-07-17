#pragma once

#include "connection_base.hpp"
#include "robot.hpp"
#include "updaters.hpp"
#include "shared.hpp"
#include "lidar.hpp"
#include "n10_lidar.hpp"
#include "robot_configuration.hpp"

#include <thread>
#include <cstdlib>

class ConnectionReal : public ConnectionBase
{
public:
    ConnectionReal(Robot* robot, Updater* updater, RobotConfiguration* conf);
    ~ConnectionReal();
    void stop() override;
    cv::Mat get_camera() override;
    std::vector<float> get_lidar() override;

    int spi_ini(const std::string& path, int channel, int speed, int mode);
    int com_ini(const std::string& path, int baud);
    std::vector<uint8_t> spi_rw(std::vector<uint8_t>& data);
    std::vector<uint8_t> com_rw(std::vector<uint8_t>& data);
    void spi_stop();
    void com_stop();

private:
    Robot* robot;
    Updater* updater;
    RobotConfiguration* conf;
    LibHolder* lib_holder;

    cv::VideoCapture* camera_instance;
    LidarBase* lidar_instance;

    std::thread* robot_info_thread;
};