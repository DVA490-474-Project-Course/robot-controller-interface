// path_planning.h
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-19
// Last modified: 2024-09-23 by Carl Larsson
// Description: Path planning header file.
// License: See LICENSE file for license details.
//==============================================================================


#ifndef ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_PATHPLANNING_H_
#define ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_PATHPLANNING_H_


// Related .h files

// C++ standard library headers
#include <algorithm>
#include <memory>
#include <mutex>

// Other .h files
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_controller/controller_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/LinearMath/Quaternion.h"

// Project .h files
#include "../individual-robot-behaviour/state.h"


namespace robot_controller_interface
{
namespace individual_robot_behaviour
{

// 
std::mutex target_reached_mutex;

// Global flag used to indicate if target has been reached or not
bool target_reached_flag = false;

//==============================================================================
//==============================================================================

} // namespace individual_robot_behaviour
} // namesapce robot_controller_interface

#endif // ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_PATHPLANNING_H_
