#include "RobotHandlerFixture.hpp"

#include <chrono>
#include <thread>

static void set_axis_speed(RobotVmxTitan* robot, float x, float y, float z)
{
    float right = -x + y / 2 + z;
    float left = x + y / 2 + z;
    float back = -y + z;

    robot->set_motor_speed_0(left);
    robot->set_motor_speed_1(right);
    robot->set_motor_speed_2(back);
}

TEST_F(RobotVmxTitanFixture, AxisSpeedDistribution)
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

TEST_F(RobotVmxTitanFixture, BaseEncodersValuesAfterRobotMoved)
{
    EXPECT_NE(0, robot->get_motor_enc_0());
    EXPECT_NE(0, robot->get_motor_enc_1());
    EXPECT_NE(0, robot->get_motor_enc_2());
}

TEST_F(RobotVmxTitanFixture, IfBaseEncodersResetWorks)
{
    robot->reset_motor_enc_0();
    robot->reset_motor_enc_1();
    robot->reset_motor_enc_2();

    EXPECT_EQ(0, robot->get_motor_enc_0());
    EXPECT_EQ(0, robot->get_motor_enc_1());
    EXPECT_EQ(0, robot->get_motor_enc_2());
}

TEST_F(RobotVmxTitanFixture, ElevatorServoInteraction)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    robot->set_servo_angle(300, 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    robot->set_servo_angle(150, 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    robot->set_servo_angle(0, 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
}

TEST_F(RobotVmxTitanFixture, GripperServoInteraction)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    robot->set_servo_angle(300, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    robot->set_servo_angle(150, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    robot->set_servo_angle(0, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
}

TEST_F(RobotVmxTitanFixture, IfLEDsAreWorkingAndAttachedCorrectly)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    robot->set_led_state(true, 1);  // Зеленый
    robot->set_led_state(false, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    robot->set_led_state(true, 2);  // Красный
    robot->set_led_state(false, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

TEST_F(RobotVmxTitanFixture, ButtonsReactionOnUserInteraction)
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

        auto vmx_flex = robot->get_vmx_flex();
        if (vmx_flex[0]) ems_pressed_once = true;
        if (vmx_flex[1]) start_pressed_once = true;
        if (vmx_flex[2]) reset_pressed_once = true;
        if (vmx_flex[3]) stop_pressed_once = true;
    }

    EXPECT_TRUE(ems_pressed_once);
    EXPECT_TRUE(start_pressed_once);
    EXPECT_TRUE(reset_pressed_once);
    EXPECT_TRUE(stop_pressed_once);
}
