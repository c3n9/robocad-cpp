#include "RobotHandlerFixture.hpp"

#include <chrono>
#include <thread>

RobotVmxTitan* RobotVmxTitanFixture::robot = nullptr;
Shufflecad* RobotVmxTitanFixture::dash = nullptr;

void RobotVmxTitanFixture::SetUpTestSuite()
{
    auto* conf = new DefaultStudicaConfiguration();
    conf->sim_log_path = "unittest-studica.log";
    robot = new RobotVmxTitan(false, conf);
    dash = new Shufflecad(robot);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void RobotVmxTitanFixture::TearDownTestSuite()
{
    delete dash;
    dash = nullptr;
    robot->stop();
    delete robot;
    robot = nullptr;
}
