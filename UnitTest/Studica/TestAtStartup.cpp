#include "RobotHandlerFixture.hpp"

TEST_F(RobotVmxTitanFixture, AnalogOutputOnRobotStartup)
{
    EXPECT_NE(0.0f, robot->get_analog_1());
    EXPECT_NE(0.0f, robot->get_analog_2());
    EXPECT_NE(0.0f, robot->get_analog_3());
    EXPECT_NE(0.0f, robot->get_analog_4());
}

TEST_F(RobotVmxTitanFixture, UltrasonicOutputOnRobotStartup)
{
    EXPECT_NE(0.0f, robot->get_us1());
    EXPECT_NE(0.0f, robot->get_us2());
}

TEST_F(RobotVmxTitanFixture, YawOutputOnRobotStartup)
{
    EXPECT_NE(0.0f, robot->get_yaw());
}

TEST_F(RobotVmxTitanFixture, IfEncodersAreZeroOnStartup)
{
    EXPECT_EQ(0, robot->get_motor_enc_0());
    EXPECT_EQ(0, robot->get_motor_enc_1());
    EXPECT_EQ(0, robot->get_motor_enc_2());
    EXPECT_EQ(0, robot->get_motor_enc_3());
}

TEST_F(RobotVmxTitanFixture, IfAllButtonsAreFalseOnRobotStartup)
{
    auto vmx_flex = robot->get_vmx_flex();
    EXPECT_FALSE(vmx_flex[0]);
    EXPECT_FALSE(vmx_flex[1]);
    EXPECT_FALSE(vmx_flex[2]);
    EXPECT_FALSE(vmx_flex[3]);
}
