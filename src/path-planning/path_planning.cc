// path_planning.cc
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-19
// Last modified: 2024-09-19 by Carl Larsson
// Description: Path planning source file, straight line for global and DWA for 
// local.
// License: See LICENSE file for license details.
//==============================================================================


// Related .h files
#include "path_planning/path_planning.h"

// Other .h files
#include "rclcpp/rclcpp.hpp"
#include "dwb_core/dwb_local_planner.hpp"
#include "nav2_core/controller.hpp"

// Project .h files
#include "common_types.h"


namespace robot_controller_interface
{
namespace path_planning
{

// Performs local path planning using DWA.
// Output: N/A
// Input: 
void local_path_planning(RobotState CurrentState, Position TargetPosition)
{
    
};

// Performs global plath planning with straight line
// Output:
// Input: 
void global_path_planning()
{
    // Create pointer to the DWA ROS2 node
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("dwb_planner_node");


};

} // namespace path_planning
} // namesapce robot_controller_interface
