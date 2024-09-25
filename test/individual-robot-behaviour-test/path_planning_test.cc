// path_planning_test.cc
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-23
// Last modified: 2024-09-25 by Carl Larsson
// Description: Test file for path planning.
// License: See LICENSE file for license details.
//==============================================================================


// Related .h files
#include "../../src/individual-robot-behaviour/path_planning.h"

// C++ standard library headers

// Other .h files
#include "gtest/gtest.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

// Project .h files

//==============================================================================
// DwbController class tests

//------------------------------------------------------------------------------

// Test Fixture for DwbController
class DwbControllerTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    // Initialize rclcpp
    rclcpp::init(0, nullptr);

    // Instantiate DwbController
    dwb_controller_ = std::make_shared<robot_controller_interface
        ::individual_robot_behaviour::DwbController>();

    // Create cmd_vel subscriber so we can check if the DWB reacts to our 
    // message containing target position and sends motor commands/velocities
    node_ = std::make_shared<rclcpp::Node>("dwb_controller_test");
    cmd_vel_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&DwbControllerTest::CmdVelCallback, this, 
            std::placeholders::_1));
  
    // To check if a message was received from cmd_vel
    message_received_ = false;
  }

  void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Store the message and set the flag
    received_msg_ = *msg;
    message_received_ = true;
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<robot_controller_interface::individual_robot_behaviour
      ::DwbController> dwb_controller_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr 
      cmd_vel_subscriber_;
  geometry_msgs::msg::Twist received_msg_;
  bool message_received_;
};

//------------------------------------------------------------------------------

// Test case for SendTargetPose
TEST_F(DwbControllerTest, SendTargetPoseTest)
{
  // Set target pose
  robot_controller_interface::individual_robot_behaviour::Pose 
      target_pose(1.0, 2.0, 2.42);

  // Send the target_pose using SendTargetPose
  dwb_controller_->SendTargetPose(target_pose);

  // Wait for the message to be sent by DWB and for us to receive it on the 
  // cmd_vel by our subscriber
  rclcpp::Rate rate(10);  // 10 Hz
  while (rclcpp::ok() && !message_received_) 
  {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(dwb_controller_->get_node_base_interface());
    rate.sleep();
  }

  // DWB sent commands on cmd_vel as a response to our message of sending
  // a target position
  SUCCEED();
}

//==============================================================================
