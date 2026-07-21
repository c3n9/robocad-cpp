#include "RobotHandlerFixture.hpp"

#include <chrono>
#include <thread>

static void set_axis_speed(RobotAlgaritm* robot, float x, float y, float z)
{
    float right = -x + y / 2 + z;
    float left = x + y / 2 + z;
    float back = -y + z;

    robot->set_motor_speed_0(left);
    robot->set_motor_speed_1(right);
    robot->set_motor_speed_2(back);
}

TEST_F(RobotAlgaritmFixture, AxisSpeedDistribution)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    set_axis_speed(robot, 50, 0, 0); // Ровно прямо
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    set_axis_speed(robot, 0, 50, 0); // Ровно вправо без скосов
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    set_axis_speed(robot, 0, 0, 50); // По часовой
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    set_axis_speed(robot, 0, 0, 0);
}

TEST_F(RobotAlgaritmFixture, BaseEncodersValuesAfterRobotMoved)
{
    EXPECT_NE(0, robot->get_motor_enc_0());
    EXPECT_NE(0, robot->get_motor_enc_1());
    EXPECT_NE(0, robot->get_motor_enc_2());
}

TEST_F(RobotAlgaritmFixture, IfBaseEncodersResetWorks)
{
    robot->reset_motor_enc_0();
    robot->reset_motor_enc_1();
    robot->reset_motor_enc_2();

    EXPECT_EQ(0, robot->get_motor_enc_0());
    EXPECT_EQ(0, robot->get_motor_enc_1());
    EXPECT_EQ(0, robot->get_motor_enc_2());
}

TEST_F(RobotAlgaritmFixture, ElevatorServoInteraction)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    robot->set_servo_angle(180, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    robot->set_servo_angle(90, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    robot->set_servo_angle(0, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
}

TEST_F(RobotAlgaritmFixture, GripperServoInteraction)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    robot->set_servo_angle(180, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    robot->set_servo_angle(90, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    robot->set_servo_angle(0, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
}

TEST_F(RobotAlgaritmFixture, IfLEDsAreWorkingAndAttachedCorrectly)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    robot->outputs[0] = true;  // Зеленый
    robot->outputs[1] = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    robot->outputs[1] = true;  // Красный
    robot->outputs[0] = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

TEST_F(RobotAlgaritmFixture, ButtonsReactionOnUserInteraction)
{
    bool ems_pressed_once = false, start_pressed_once = false;
    bool reset_pressed_once = false, stop_pressed_once = false;

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (!ems_pressed_once || !start_pressed_once || !reset_pressed_once || !stop_pressed_once)
    {
        if (std::chrono::steady_clock::now() > deadline)
        {
            FAIL() << "Test execution timed out after 30000 milliseconds";
            return;
        }

        auto inputs = robot->get_inputs();
        if (inputs[0]) ems_pressed_once = true;
        if (inputs[1]) start_pressed_once = true;
        if (inputs[2]) reset_pressed_once = true;
        if (inputs[3]) stop_pressed_once = true;
    }

    EXPECT_TRUE(ems_pressed_once);
    EXPECT_TRUE(start_pressed_once);
    EXPECT_TRUE(reset_pressed_once);
    EXPECT_TRUE(stop_pressed_once);
}
