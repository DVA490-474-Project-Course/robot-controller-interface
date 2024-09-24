// state_test.cc
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-23
// Last modified: 2024-09-23 by Carl Larsson
// Description: Test file for state source and header files.
// License: See LICENSE file for license details.
//==============================================================================


// Related .h files
#include "../../src/individual-robot-behaviour/state.h"

// C++ standard library headers
#include <cmath>

// Other .h files
#include "gtest/gtest.h"

// Project .h files
#include "../../src/common_types.h"


//==============================================================================
// Tests for Pose class

// Test fixture for Pose class
class PoseClassTest : public ::testing::Test {
 protected:
  // This will run before each test (TEST_F)
  void SetUp() override {
    // Initialize Pose
    robot_controller_interface::individual_robot_behaviour::Pose pose_;
  }

  // Test object
  robot_controller_interface::individual_robot_behaviour::Pose pose_;
};

// Test case for Pose class, initial values (assuming default constructor was 
// used) and GetX,GetY,GetTheta
TEST_F(PoseClassTest, InitialValues)
{
  EXPECT_FLOAT_EQ(pose_.GetX(), 0.0);

  EXPECT_FLOAT_EQ(pose_.GetY(), 0.0);

  EXPECT_FLOAT_EQ(pose_.GetTheta(), 0.0);
}

// Test case for Pose class, setting values inside boundary
TEST_F(PoseClassTest, SettingValues)
{
  double x = 2.0;
  pose_.SetX(x);
  EXPECT_FLOAT_EQ(pose_.GetX(), x);

  double y = 5.2;
  pose_.SetY(y);
  EXPECT_FLOAT_EQ(pose_.GetY(), y);

  double theta = 0.5;
  pose_.SetTheta(theta);
  EXPECT_FLOAT_EQ(pose_.GetTheta(), theta);
}

// Test case for Pose class, setting values outside boundary
TEST_F(PoseClassTest, BoundaryValues)
{
  // Positive
  double x = robot_controller_interface::PlayingField::kFrameX/2;
  pose_.SetX(x + 10);
  EXPECT_FLOAT_EQ(pose_.GetX(), x);

  double y = robot_controller_interface::PlayingField::kFrameY/2;
  pose_.SetY(y + 10);
  EXPECT_FLOAT_EQ(pose_.GetY(), y);

  // Negative
  x = -(robot_controller_interface::PlayingField::kFrameX/2);
  pose_.SetX(x - 10);
  EXPECT_FLOAT_EQ(pose_.GetX(), x);

  y = -(robot_controller_interface::PlayingField::kFrameY/2);
  pose_.SetY(y - 10);
  EXPECT_FLOAT_EQ(pose_.GetY(), y);

  // Wrap
  double theta = 24.6;
  pose_.SetTheta(theta);
  EXPECT_FLOAT_EQ(pose_.GetTheta(), atan2(sin(theta), cos(theta)));
}

// Test case for Pose class inequality operator
// Doesn't have to be F but prefer all grouped together
TEST_F(PoseClassTest, InequalityOperator)
{
  robot_controller_interface::individual_robot_behaviour::Pose a;
  a.SetX(1.0);
  a.SetY(1.0);
  a.SetTheta(1.0);

  robot_controller_interface::individual_robot_behaviour::Pose b;
  b.SetX(1.0);
  b.SetY(1.0);
  b.SetTheta(1.0);

  
  // Exactly equal
  EXPECT_FALSE(a != b);

  // Difference exactly equal to tolerance
  a.SetX(1.0 + 1e-6);
  EXPECT_FALSE(a != b);

  // Difference slightly outside tolerance
  a.SetX(1.0 + 1e-5);
  EXPECT_TRUE(a != b);

  // Difference slightly within tolerance
  a.SetX(1.0 + 1e-7);
  EXPECT_FALSE(a != b);
}

//==============================================================================
// Tests for RobotState class

//==============================================================================
