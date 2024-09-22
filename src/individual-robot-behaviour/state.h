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
struct Pose
{
    double x;
    double y;
    double theta;
};

//==============================================================================

// Class describing the current state of the robot
// Neither copyable nor move-only.
class RobotState
{
public:
    double x;
    double y;
    double theta;
    bool ball;
};

//==============================================================================

// Global variable for keeping track of current robot state
// Needs to be global since its used between multiple threads to keep track
// of current robot state.
RobotState current_state = RobotState{0,0,0,false};

//==============================================================================

} // namespace individual_robot_behaviour
} // namespace robot_controller_interface

#endif // ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_PATHPLANNING_H_
