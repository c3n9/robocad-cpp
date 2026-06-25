#include "connection_real.hpp"

#include "lidar.hpp"
#include <opencv2/opencv.hpp>
#include <cstdlib>

ConnectionReal::ConnectionReal(Robot* robot, Updater* updater, RobotConfiguration* conf)
    : robot(robot), updater(updater), conf(conf)
{
    lib_holder = new LibHolder(conf->lib_holder_first_path);

    try
    {
        camera_instance = new cv::VideoCapture(conf->camera_index);
    }
    catch (const std::exception& e)
    {
        robot->write_log(std::string("Error while creating camera instance: ") + e.what());
    }

    try
    {
        lidar_instance = new YDLidarX2(robot, conf->lidar_port);
        lidar_instance->connect();
        lidar_instance->start_scan();
    }
    catch (const std::exception& e)
    {
        robot->write_log(std::string("Error while creating lidar instance: ") + e.what());
    }

    // pi-blaster
    if (conf->with_pi_blaster)
    {
        system("sudo /home/pi/pi-blaster/pi-blaster --pcm");
    }
    // robot info thread (kept joinable so stop()/destructor can join it)
    robot_info_thread = new std::thread(&Updater::updater, updater);
}

ConnectionReal::~ConnectionReal()
{
    // make sure the info thread is stopped and joined before we tear down
    if (robot_info_thread && robot_info_thread->joinable())
    {
        if (updater) updater->stop_robot_info_thread = true;
        robot_info_thread->join();
    }
    delete camera_instance;
    camera_instance = NULL;
    delete lidar_instance;
    lidar_instance = NULL;
    delete robot_info_thread;
    robot_info_thread = NULL;
}

void ConnectionReal::stop()
{
    if (lidar_instance)
    {
        lidar_instance->stop_scan();
        lidar_instance->disconnect();
    }

    updater->stop_robot_info_thread = true;
    if (robot_info_thread && robot_info_thread->joinable())
        robot_info_thread->join();
}

cv::Mat ConnectionReal::get_camera()
{
    cv::Mat frame;
    try
    {
        camera_instance->read(frame);
    }
    catch (const std::exception& e)
    {
        // pass
    }
    return frame;
}

std::vector<float> ConnectionReal::get_lidar()
{
    try
    {
        return lidar_instance->get_data();
    }
    catch (const std::exception& e)
    {
        // pass
    }
    return std::vector<float>();
}

int ConnectionReal::spi_ini(const std::string& path, int channel, int speed, int mode)
{
    return lib_holder->init_spi(path, channel, speed, mode);
}

int ConnectionReal::com_ini(const std::string& path, int baud)
{
    return lib_holder->init_usb(path, baud);
}

std::vector<uint8_t> ConnectionReal::spi_rw(std::vector<uint8_t>& data)
{
    std::vector<uint8_t> buffer = data;
    unsigned char* ptr = buffer.data();
    unsigned int len = buffer.size();

    unsigned char* result_ptr = lib_holder->rw_spi(ptr, len);
    return std::vector<uint8_t>(result_ptr, result_ptr + len);
}

std::vector<uint8_t> ConnectionReal::com_rw(std::vector<uint8_t>& data)
{
    std::vector<uint8_t> buffer = data;
    unsigned char* ptr = buffer.data();
    unsigned int len = buffer.size();

    unsigned char* result_ptr = lib_holder->rw_usb(ptr, len);
    return std::vector<uint8_t>(result_ptr, result_ptr + len);
}

void ConnectionReal::spi_stop()
{
    return lib_holder->stop_spi();
}

void ConnectionReal::com_stop()
{
    return lib_holder->stop_usb();
}