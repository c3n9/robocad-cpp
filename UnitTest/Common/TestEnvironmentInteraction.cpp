#include "RobotHandlerFixture.hpp"

#include <chrono>
#include <thread>

static void set_axis_speed(CommonRobot* robot, float x, float z)
{
    float right = -x + z;
    float left = x + z;

    robot->set_motor_speed_0(left);
    robot->set_motor_speed_1(left);
    robot->set_motor_speed_2(left);

    robot->set_motor_speed_3(right);
    robot->set_motor_speed_4(right);
    robot->set_motor_speed_5(right);
}

// Tests below run in declaration order (gtest preserves registration order
// within a test suite by default, no shuffling) - this replaces the explicit
// [Priority(n)] ordering used in the C# port for the same purpose.

TEST_F(CommonRobotFixture, AxisSpeedDistribution)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    set_axis_speed(robot, 50, 0); // Прямо, но будет скос вправо
    std::this_thread::sleep_for(std::chrono::milliseconds(7000));
    set_axis_speed(robot, 0, 50); // По часовой
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    set_axis_speed(robot, 0, 0);
}

TEST_F(CommonRobotFixture, BaseEncodersValuesAfterRobotMoved)
{
    EXPECT_NE(0, robot->get_motor_enc_0());
    EXPECT_NE(0, robot->get_motor_enc_1());
    EXPECT_NE(0, robot->get_motor_enc_2());
    EXPECT_NE(0, robot->get_motor_enc_3());
    EXPECT_NE(0, robot->get_motor_enc_4());
    EXPECT_NE(0, robot->get_motor_enc_5());
}

TEST_F(CommonRobotFixture, IfBaseEncodersResetWorks)
{
    robot->reset_motor_enc_0();
    robot->reset_motor_enc_1();
    robot->reset_motor_enc_2();
    robot->reset_motor_enc_3();
    robot->reset_motor_enc_4();
    robot->reset_motor_enc_5();

    EXPECT_EQ(0, robot->get_motor_enc_0());
    EXPECT_EQ(0, robot->get_motor_enc_1());
    EXPECT_EQ(0, robot->get_motor_enc_2());
    EXPECT_EQ(0, robot->get_motor_enc_3());
    EXPECT_EQ(0, robot->get_motor_enc_4());
    EXPECT_EQ(0, robot->get_motor_enc_5());
}

TEST_F(CommonRobotFixture, ElevatingArmServoInteraction)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    robot->set_servo_angle(300, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));
    robot->set_servo_angle(150, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    robot->set_servo_angle(0, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
}

TEST_F(CommonRobotFixture, GripperServoInteraction)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    robot->set_servo_angle(300, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    robot->set_servo_angle(150, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    robot->set_servo_angle(0, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
}

TEST_F(CommonRobotFixture, ButtonsReactionOnUserInteraction)
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

        auto buttons = robot->get_buttons();
        if (buttons[0]) ems_pressed_once = true;
        if (buttons[1]) start_pressed_once = true;
        if (buttons[2]) reset_pressed_once = true;
        if (buttons[3]) stop_pressed_once = true;
    }

    EXPECT_TRUE(ems_pressed_once);
    EXPECT_TRUE(start_pressed_once);
    EXPECT_TRUE(reset_pressed_once);
    EXPECT_TRUE(stop_pressed_once);
}
