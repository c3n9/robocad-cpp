#include "RobotHandlerFixture.hpp"

#include <chrono>
#include <thread>

CommonRobot* CommonRobotFixture::robot = nullptr;
Shufflecad* CommonRobotFixture::dash = nullptr;

void CommonRobotFixture::SetUpTestSuite()
{
    auto* conf = new DefaultCommonConfiguration();
    conf->sim_log_path = "unittest-common.log";
    robot = new CommonRobot(false, conf);
    dash = new Shufflecad(robot);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void CommonRobotFixture::TearDownTestSuite()
{
    delete dash;   // joins the Shufflecad worker threads and frees ports 63253-63259
    dash = nullptr;
    robot->stop();
    delete robot;
    robot = nullptr;
}
