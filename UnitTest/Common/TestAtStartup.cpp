#include "RobotHandlerFixture.hpp"

TEST_F(CommonRobotFixture, AnalogOutputOnRobotStartup)
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

TEST_F(CommonRobotFixture, UltrasonicOutputOnRobotStartup)
{
    // EXPECT_* (unlike ASSERT_*) records a failure and keeps going, so all
    // four checks are reported even if one fails - the same intent as the
    // C# version's Assert.Multiple wrapper.
    EXPECT_NE(0.0f, robot->get_us1());
    EXPECT_NE(0.0f, robot->get_us2());
    EXPECT_NE(0.0f, robot->get_us3());
    EXPECT_NE(0.0f, robot->get_us4());
}

TEST_F(CommonRobotFixture, YawOutputOnRobotStartup)
{
    EXPECT_NE(0.0f, robot->get_yaw());
}

TEST_F(CommonRobotFixture, IfEncodersAreZeroOnStartup)
{
    EXPECT_EQ(0, robot->get_motor_enc_0());
    EXPECT_EQ(0, robot->get_motor_enc_1());
    EXPECT_EQ(0, robot->get_motor_enc_2());
    EXPECT_EQ(0, robot->get_motor_enc_3());
    EXPECT_EQ(0, robot->get_motor_enc_4());
    EXPECT_EQ(0, robot->get_motor_enc_5());
    EXPECT_EQ(0, robot->get_motor_enc_6());
    EXPECT_EQ(0, robot->get_motor_enc_7());
}

TEST_F(CommonRobotFixture, IfAllButtonsAreFalseOnRobotStartup)
{
    auto buttons = robot->get_buttons();
    EXPECT_FALSE(buttons[0]);
    EXPECT_FALSE(buttons[1]);
    EXPECT_FALSE(buttons[2]);
    EXPECT_FALSE(buttons[3]);
}
