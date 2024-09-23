// state.h
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-22
// Last modified: 2024-09-22 by Carl Larsson
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


namespace robot_controller_interface
{
namespace individual_robot_behaviour
{

//==============================================================================

// Struct for a pose in 2D space.
class Pose
{
 public:
    double x_;
    double y_;
    double theta_;

    // != operator for this class
    bool operator!=(const Pose& other) const
    {
        return (std::fabs(x_ - other.x_) > tolerance_) ||
               (std::fabs(y_ - other.y_) > tolerance_) ||
               (std::fabs(theta_ - other.theta_) > tolerance_);
    }
 private:
    const double tolerance_ = 1e-6;
};

//==============================================================================

// Class describing the current state of the robot
// Neither copyable nor move-only.
class RobotState
{
 public:
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
RobotState current_state = RobotState{0,0,0,false};
// Global mutex to protect target_pose
std::mutex target_mutex;

//==============================================================================

} // namespace individual_robot_behaviour
} // namespace robot_controller_interface

#endif // ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_PATHPLANNING_H_
