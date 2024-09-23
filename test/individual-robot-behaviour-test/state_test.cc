// state_test.cc
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-23
// Last modified: 2024-09-23 by Carl Larsson
// Description: Test file for state source and header file.
// License: See LICENSE file for license details.
//==============================================================================


// Related .h files
#include "../../src/individual-robot-behaviour/state.h"

// C++ standard library headers

// Other .h files
#include <gtest/gtest.h>

// Project .h files


//==============================================================================

// Test case for Pose class inequality operator
TEST(PoseClassTest, InequalityOperator)
{
  robot_controller_interface::individual_robot_behaviour::Pose a;
  a.x_ = 1.0;
  a.y_ = 1.0;
  a.theta_ = 1.0;

  robot_controller_interface::individual_robot_behaviour::Pose b;
  b.x_ = 1.0;
  b.y_ = 1.0;
  b.theta_ = 1.0;

  
  // Exactly equal
  EXPECT_FALSE(a != b);

  // Difference exactly equal to tollerance
  a.x_ = 1.0 + 1e-6;
  EXPECT_FALSE(a != b);

  // Difference slightly outside tollerance
  a.x_ = 1.0 + 1e-5;
  EXPECT_TRUE(a != b);

  // Difference slightly within tollerance
  a.x_ = 1.0 + 1e-7;
  EXPECT_FALSE(a != b);
}

//==============================================================================
