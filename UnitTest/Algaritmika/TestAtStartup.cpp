#include "RobotHandlerFixture.hpp"

TEST_F(RobotAlgaritmFixture, AnalogOutputOnRobotStartup)
{
    EXPECT_NE(0.0f, robot->get_analog_1());
    EXPECT_NE(0.0f, robot->get_analog_2());
    EXPECT_NE(0.0f, robot->get_analog_3());
    EXPECT_NE(0.0f, robot->get_analog_4());
    EXPECT_NE(0.0f, robot->get_analog_5());
    EXPECT_NE(0.0f, robot->get_analog_6());
    EXPECT_NE(0.0f, robot->get_analog_7());
    EXPECT_NE(0.0f, robot->get_analog_8());
}

TEST_F(RobotAlgaritmFixture, UltrasonicOutputOnRobotStartup)
{
    EXPECT_NE(0.0f, robot->get_us1());
    EXPECT_NE(0.0f, robot->get_us2());
    EXPECT_NE(0.0f, robot->get_us3());
    EXPECT_NE(0.0f, robot->get_us4());
}

TEST_F(RobotAlgaritmFixture, YawOutputOnRobotStartup)
{
    EXPECT_NE(0.0f, robot->get_yaw());
}

TEST_F(RobotAlgaritmFixture, RollOutputOnRobotStartup)
{
    EXPECT_NE(0.0f, robot->get_roll());
}

TEST_F(RobotAlgaritmFixture, PitchOutputOnRobotStartup)
{
    EXPECT_NE(0.0f, robot->get_pitch());
}

TEST_F(RobotAlgaritmFixture, IfEncodersAreZeroOnStartup)
{
    EXPECT_EQ(0, robot->get_motor_enc_0());
    EXPECT_EQ(0, robot->get_motor_enc_1());
    EXPECT_EQ(0, robot->get_motor_enc_2());
    EXPECT_EQ(0, robot->get_motor_enc_3());
}

TEST_F(RobotAlgaritmFixture, IfAllButtonsAreFalseOnRobotStartup)
{
    auto inputs = robot->get_inputs();
    EXPECT_FALSE(inputs[0]);
    EXPECT_FALSE(inputs[1]);
    EXPECT_FALSE(inputs[2]);
    EXPECT_FALSE(inputs[3]);
}
