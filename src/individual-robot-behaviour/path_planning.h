// path_planning.h
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-19
// Last modified: 2024-09-22 by Carl Larsson
// Description: Path planning header file.
// License: See LICENSE file for license details.
//==============================================================================


#ifndef ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_PATHPLANNING_H_
#define ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_PATHPLANNING_H_


// Related .h files

// C++ standard library headers
#include <algorithm>
#include <memory>

// Other .h files
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_controller/controller_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/LinearMath/Quaternion.h"

// Project .h files
#include "../common_types.h"


namespace robot_controller_interface
{
namespace individual_robot_behaviour
{


// Struct for a pose in 2D space.
struct Pose
{
    double x;
    double y;
    double theta;
};


} // namespace individual_robot_behaviour
} // namespace robot_controller_interface

#endif // ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_PATHPLANNING_H_
