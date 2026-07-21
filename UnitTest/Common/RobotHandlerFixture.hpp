#pragma once

#include "common.hpp"
#include "shufflecad.hpp"

#include <gtest/gtest.h>

// Shared fixture for the Common kit, analogous to RobotHandlerFixture +
// ICollectionFixture<RobotHandlerFixture> in the C# UnitTest project: the
// robot and its Shufflecad connection are created once for the whole
// CommonRobotFixture test suite (SetUpTestSuite/TearDownTestSuite run once,
// not per test) and shared by every TEST_F below, whether they live in
// TestAtStartup.cpp or TestEnvironmentInteraction.cpp.
//
// These are integration/HIL tests: Analog/Us/Yaw/encoder/button values only
// become non-zero once a real robot or the Shufflecad companion app is
// connected and feeding data over TCP (ports 63253-63259). Without that,
// the *AtStartup assertions on non-zero values will fail and
// ButtonsReactionOnUserInteraction will time out - same as the C# port.
class CommonRobotFixture : public ::testing::Test
{
protected:
    static CommonRobot* robot;
    static Shufflecad* dash;

    static void SetUpTestSuite();
    static void TearDownTestSuite();
};
