// state.cc
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-22
// Last modified: 2024-09-24 by Carl Larsson
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

// Global variable for keeping track of current robot state
// Needs to be global since its used between multiple threads to keep track
// of current robot state.
RobotState current_state;
// Global mutex to protect target_pose
std::mutex target_mutex;

//==============================================================================
// Class for a pose in 2D space.
// Neither copyable nor move-only.

// Default constructor
Pose::Pose() : x_(0.0), y_(0.0), theta_(0.0) {}

// Parameterized constructor
Pose::Pose(double x, double y, double theta) 
{
  SetX(x);
  SetY(y);
  SetTheta(theta);
}

// Get the members value
double Pose::GetX() const { return x_; }
double Pose::GetY() const { return y_; }
double Pose::GetTheta() const { return theta_; }

// Set the members values
// Values must be within playing field
void Pose::SetX(double x) 
{
  x_ = (x > PlayingField::kFrameX / 2) ? PlayingField::kFrameX / 2 :
       (x < -PlayingField::kFrameX / 2) ? -PlayingField::kFrameX / 2 :
       x;
}
void Pose::SetY(double y) 
{
  y_ = (y > PlayingField::kFrameY / 2) ? PlayingField::kFrameY / 2 :
       (y < -PlayingField::kFrameY / 2) ? -PlayingField::kFrameY / 2 :
       y;
}
void Pose::SetTheta(double theta) 
{
  // Wrap angle to [-pi, pi]
  theta_ = atan2(sin(theta), cos(theta));
}

// != operator for this class
bool Pose::operator!=(const Pose& other) const 
{
  return (std::fabs(x_ - other.x_) > tolerance_) ||
         (std::fabs(y_ - other.y_) > tolerance_) ||
         (std::fabs(theta_ - other.theta_) > tolerance_);
}

//==============================================================================
// Class describing the current state of the robot
// Neither copyable nor move-only.

// Default constructor
RobotState::RobotState() : x_(0.0), y_(0.0), theta_(0.0), ball_(false) {}

// Parameterized constructor
RobotState::RobotState(double x, double y, double theta, bool ball) 
{
  SetX(x);
  SetY(y);
  SetTheta(theta);
  SetBall(ball);
}

// Get the members value
double RobotState::GetX() const { return x_; }
double RobotState::GetY() const { return y_; }
double RobotState::GetTheta() const { return theta_; }
bool RobotState::GetBall() const { return ball_; }

// Set the members values
void RobotState::SetX(double x) 
{ 
  x_ = x; 
}
void RobotState::SetY(double y) 
{ 
  y_ = y; 
}
void RobotState::SetTheta(double theta)
{
  // Wrap angle to [-pi, pi]
  theta_ = atan2(sin(theta), cos(theta));
}
void RobotState::SetBall(bool ball) 
{ 
  ball_ = ball; 
}

//==============================================================================

// Odom Subscriber
// Neither copyable nor move-only.
OdomSubscriber::OdomSubscriber()
  : rclcpp_lifecycle::LifecycleNode("odom_subscriber")
{
  // Subscribe to odom topic
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&OdomSubscriber::OdomCallback, this, 
          std::placeholders::_1));
}

// Description: Odometry callback function, stores current state in global
// variable current_state.
// Use: use as argument when creating odometry subscriber.
// Input: const shared pointer to msg containing odometry data. 
// Output N/A
// Return value: void
void OdomSubscriber::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr 
    msg) const
{
  // Store globally
  current_state.SetX(msg->pose.pose.position.x);
  current_state.SetY(msg->pose.pose.position.y);
  current_state.SetTheta(tf2::getYaw(msg->pose.pose.orientation));
}

//==============================================================================

} // namespace individual_robot_behaviour
} // namesapce robot_controller_interface
