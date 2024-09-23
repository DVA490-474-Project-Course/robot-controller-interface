// state.cc
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-22
// Last modified: 2024-09-23 by Carl Larsson
// Description: Robot state source file. Odometry, drift correction etc.
// License: See LICENSE file for license details.
//==============================================================================


// Related .h files
#include "../individual-robot-behaviour/state.h"

// C++ standard library headers

// Other .h files
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_controller/controller_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Project .h files


namespace robot_controller_interface
{
namespace individual_robot_behaviour
{

//==============================================================================

// Odom Subscriber
// Neither copyable nor move-only.
OdomSubscriber::OdomSubscriber()
  : rclcpp_lifecycle::LifecycleNode("odom_subscriber")
{
  // Subscribe to odom topic
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&OdomSubscriber::OdomCallback, this, std::placeholders::_1));
}

// Description: Odometry callback function, stores current state in global
// variable current_state.
// Use: use as argument when creating odometry subscriber.
// Input: const shared pointer to msg containing odometry data. 
// Output N/A
// Return value: void
void OdomSubscriber::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) const
{
  // Store globally
  current_state.x_ = msg->pose.pose.position.x;
  current_state.y_ = msg->pose.pose.position.y;
  current_state.theta_ = tf2::getYaw(msg->pose.pose.orientation);
}

//==============================================================================

} // namespace individual_robot_behaviour
} // namesapce robot_controller_interface
