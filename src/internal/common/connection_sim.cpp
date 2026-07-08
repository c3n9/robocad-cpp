#include "connection_sim.hpp"

#include <opencv2/opencv.hpp>
#include <cstdlib>

ConnectionSim::ConnectionSim(Robot* robot)
    : robot(robot)
{
    talk_channel = new TalkPort(robot, port_set_data);
    talk_channel->start_talking();
    listen_channel = new ListenPort(robot, port_get_data);
    listen_channel->start_listening();
    camera_channel = new ListenPort(robot, port_camera);
    camera_channel->start_listening();
}

ConnectionSim::~ConnectionSim()
{
    delete talk_channel;
    talk_channel = NULL;
    delete listen_channel;
    listen_channel = NULL;
    delete camera_channel;
    camera_channel = NULL;
}

void ConnectionSim::stop()
{
    if (talk_channel)
        talk_channel->stop_talking();
    if (listen_channel)
        listen_channel->stop_listening();
    if (camera_channel)
        camera_channel->stop_listening();
}

cv::Mat ConnectionSim::get_camera()
{
    if (!camera_channel)
        return cv::Mat();
    std::vector<uint8_t> data = camera_channel->get_bytes_safe();
    if (data.size() == 921600)
    {
        cv::Mat img(480, 640, CV_8UC3, data.data());
        cv::Mat img_bgr;
        cv::cvtColor(img, img_bgr, cv::COLOR_RGB2BGR);
        cv::Mat rotated, flipped;
        cv::rotate(img_bgr, rotated, cv::ROTATE_180);
        cv::flip(rotated, flipped, 1);
        return flipped;
    }
    return cv::Mat();
}

std::vector<float> ConnectionSim::get_lidar()
{
    return std::vector<float>();
}

void ConnectionSim::set_data(std::vector<uint8_t> data)
{
    talk_channel->set_bytes_safe(data);
}

std::vector<uint8_t> ConnectionSim::get_data()
{
    return listen_channel->get_bytes_safe();
}