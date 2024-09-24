// state.h
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-22
// Last modified: 2024-09-23 by Carl Larsson
// Description: Robot state header file.
// License: See LICENSE file for license details.
//==============================================================================


#ifndef ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_STATE_H_
#define ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_STATE_H_


// Related .h files

// C++ standard library headers
#include <cmath>
#include <mutex>

// Other .h files
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Project .h files
#include "../common_types.h"


namespace robot_controller_interface
{
namespace individual_robot_behaviour
{

//==============================================================================

// Class for a pose in 2D space.
// Neither copyable nor move-only.
class Pose
{
 public:
  // Default constructor
  Pose() : x_(0.0), y_(0.0), theta_(0.0) {}

  // Parameterized constructor
  Pose(double x, double y, double theta)
  {
    SetX(x);
    SetY(y);
    SetTheta(theta);
  }

  // Get the members value
  double GetX() const { return x_; }
  double GetY() const { return y_; }
  double GetTheta() const { return theta_; }

  // Set the members values
  // They must be within the playing field
  void SetX(double x)
  {
    x_ = (x > PlayingField::kFrameX/2) ? PlayingField::kFrameX/2 :
        (x < -PlayingField::kFrameX/2) ? -PlayingField::kFrameX/2 :
            x;
  }
  void SetY(double y)
  {
    y_ = (y > PlayingField::kFrameY/2) ? PlayingField::kFrameY/2 :
        (y < -PlayingField::kFrameY/2) ? -PlayingField::kFrameY/2 :
            y;
  }
  void SetTheta(double theta)
  {
    // Wrap angle to [-pi,pi]
    theta_ = atan2(sin(theta), cos(theta));
  }

  // != operator for this class
  bool operator!=(const Pose& other) const
  {
    return (std::fabs(x_ - other.x_) > tolerance_) ||
           (std::fabs(y_ - other.y_) > tolerance_) ||
           (std::fabs(theta_ - other.theta_) > tolerance_);
  }
 private:
  // Member variables, can only be accessed with Get and Set functions above
  double x_;
  double y_;
  double theta_;

  // Tolerance for != operation
  const double tolerance_ = 1e-6;
};

//==============================================================================

// Class describing the current state of the robot
// Neither copyable nor move-only.
class RobotState
{
 public:
  // Default constructor
  RobotState() : x_(0.0), y_(0.0), theta_(0.0), ball_(false) {}

  // Parameterized constructor
  RobotState(double x, double y, double theta, bool ball)
  {
    SetX(x);
    SetY(y);
    SetTheta(theta);
    SetBall(ball);
  }

  // Get the members value
  double GetX() const { return x_; }
  double GetY() const { return y_; }
  double GetTheta() const { return theta_; }
  bool GetBall() const { return ball_; }

  void SetX(double x)
  {
    x_ = x;
  }
  void SetY(double y)
  {
    y_ = y;
  }
  void SetTheta(double theta)
  {
    // Wrap angle to [-pi,pi]
    theta_ = atan2(sin(theta), cos(theta));
  }
  void SetBall(bool ball)
  {
    ball_ = ball;
  }
 private:
  // Member variables, can only be accessed with Get and Set functions above
  double x_;
  double y_;
  double theta_;
  bool ball_;
};

//==============================================================================

// Odom Subscriber
// Neither copyable nor move-only.
class OdomSubscriber : public rclcpp_lifecycle::LifecycleNode
{
 public:
  OdomSubscriber();

 private:
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) const;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
};

//==============================================================================

// Global variable for keeping track of current robot state
// Needs to be global since its used between multiple threads to keep track
// of current robot state.
extern RobotState current_state;
// Global mutex to protect target_pose
extern std::mutex target_mutex;

//==============================================================================

} // namespace individual_robot_behaviour
} // namespace robot_controller_interface

#endif // ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_PATHPLANNING_H_
