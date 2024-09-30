// state.h
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-22
// Last modified: 2024-09-30 by Carl Larsson
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
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

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
  Pose();

  // Parameterized constructor
  Pose(double x, double y, double theta);

  // Get the members value
  double GetX() const;
  double GetY() const;
  double GetTheta() const;

  // Set the members values
  // Only allows values within playinfield
  void SetX(double x);
  void SetY(double y);
  void SetTheta(double theta);

  // != operator for this class
  bool operator!=(const Pose& other) const;

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
  RobotState();

  // Parameterized constructor
  RobotState(double x, double y, double theta, bool ball);

  // Get the members' value
  double GetX() const;
  double GetY() const;
  double GetTheta() const;
  bool GetBall() const;

  // Set the members' values
  void SetX(double x);
  void SetY(double y);
  void SetTheta(double theta);
  void SetBall(bool ball);

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
class OdomSubscriber : public rclcpp::Node
{
 public:
  OdomSubscriber();

 private:
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) const;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
};

//==============================================================================

// Calculates angle between current and target pose assume a playing field which
// follows unit circle coordinations with four quadrants
double CalculateAngle(double current_x, double current_y, 
    double target_x, double target_y);

//==============================================================================

// Global variable for keeping track of current robot state
// Needs to be global since its used between multiple threads to keep track
// of current robot state.
extern RobotState current_state;
// Global mutex to protect current_state (global)
extern std::mutex current_state_mutex;

//==============================================================================

} // namespace individual_robot_behaviour
} // namespace robot_controller_interface

#endif // ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_PATHPLANNING_H_
