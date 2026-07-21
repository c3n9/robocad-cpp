#include "RobotHandlerFixture.hpp"

#include <chrono>
#include <thread>

RobotAlgaritm* RobotAlgaritmFixture::robot = nullptr;
Shufflecad* RobotAlgaritmFixture::dash = nullptr;

void RobotAlgaritmFixture::SetUpTestSuite()
{
    auto* conf = new DefaultAlgaritmConfiguration();
    conf->sim_log_path = "unittest-algaritmika.log";
    robot = new RobotAlgaritm(false, conf);
    dash = new Shufflecad(robot);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void RobotAlgaritmFixture::TearDownTestSuite()
{
    delete dash;
    dash = nullptr;
    robot->stop();
    delete robot;
    robot = nullptr;
}
