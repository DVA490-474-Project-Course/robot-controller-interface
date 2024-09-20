// path_planning.cc
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-19
// Last modified: 2024-09-20 by Carl Larsson
// Description: Path planning source file, global path planning is not
// necessary, passing the desitnation position instantly and letting DWA (local
// path planning) handle the rest is an acceptable simplification in the 
// SSL-RoboCup environment.
// License: See LICENSE file for license details.
//==============================================================================


// Related .h files
#include "../path-planning/path_planning.h"

// C++ standard library headers
#include <functional>
#include <memory>

// Other .h files
#include "rclcpp/rclcpp.hpp"
#include "dwb_core/dwb_local_planner.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Project .h files
#include "../common_types.h"


namespace robot_controller_interface
{
namespace path_planning
{

// Performs local path planning using DWA.
// Output: N/A
// Input: 
void local_path_planning(RobotState CurrentState, Position TargetPosition)
{
    // Initialize rclcpp in main

    // Create pointer to the DWA ROS2 node
    std::shared_ptr<DwbNode> dwb_node = std::make_shared<DwbNode>();

    // Define a single goal pose
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = rclcpp::Clock().now();
    goal_pose.header.frame_id = "odom";  // TODO maybe have a map frame?
    // Set goal position
    goal_pose.pose.position.x = TargetPosition.x;
    goal_pose.pose.position.y = TargetPosition.y;
    goal_pose.pose.orientation.w = TargetPosition.theta;


    // Publish target position
    (*dwb_node).publish_single_goal(goal_pose);
    // Start control loop which handles velocity publishing
    (*dwb_node).StartControlLoop();

    //rclcpp::spin(dwb_node);

    // Shutdown rclcpp in main
};

} // namespace path_planning
} // namesapce robot_controller_interface
