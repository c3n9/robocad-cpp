#pragma once

#include "algaritm.hpp"
#include "shufflecad.hpp"

#include <gtest/gtest.h>

// See UnitTest/Common/RobotHandlerFixture.hpp for the rationale: one robot +
// Shufflecad connection shared by every TEST_F(RobotAlgaritmFixture, ...) in
// this suite, across both TestAtStartup.cpp and TestEnvironmentInteraction.cpp.
class RobotAlgaritmFixture : public ::testing::Test
{
protected:
    static RobotAlgaritm* robot;
    static Shufflecad* dash;

    static void SetUpTestSuite();
    static void TearDownTestSuite();
};
