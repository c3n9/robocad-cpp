#pragma once

#include "studica.hpp"
#include "shufflecad.hpp"

#include <gtest/gtest.h>

// See UnitTest/Common/RobotHandlerFixture.hpp for the rationale: one robot +
// Shufflecad connection shared by every TEST_F(RobotVmxTitanFixture, ...) in
// this suite, across both TestAtStartup.cpp and TestEnvironmentInteraction.cpp.
class RobotVmxTitanFixture : public ::testing::Test
{
protected:
    static RobotVmxTitan* robot;
    static Shufflecad* dash;

    static void SetUpTestSuite();
    static void TearDownTestSuite();
};
