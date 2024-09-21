// path_planning.cc
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-19
// Last modified: 2024-09-21 by Carl Larsson
// Description: Path planning source file, global path planning is not
// necessary, passing the desitnation position instantly and letting DWA (local
// path planning) handle the rest is an acceptable simplification in the 
// SSL-RoboCup environment.
// License: See LICENSE file for license details.
//==============================================================================


// Related .h files
#include "../path-planning/path_planning.h"

// C++ standard library headers
#include <algorithm>
#include <memory>

// Other .h files
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "tf2/LinearMath/Quaternion.h"

// Project .h files
#include "../common_types.h"


namespace robot_controller_interface
{
namespace path_planning
{

//==============================================================================
// DWB controller
class DwbNavController : public rclcpp::Node 
{
//------------------------------------------------------------------------------
 public:
  DwbNavController() : Node("dwb_nav_controller")
  {
    // Variables

    // Subscribe to odom topic
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&DwbNavController::OdomCallback, this, std::placeholders::_1));

    // Define the action client for sending target position to controller
    controller_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(
      this, "follow_path");
  }

//------------------------------------------------------------------------------
//Functions

  // Send the target pose to DWB controller
  void SendTargetPose(Position target_position) 
  {
    // Create message which will be sent
    nav2_msgs::action::FollowPath::Goal goal_msg;

    // Message contents (target position)
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "map"; // Change in the future?
    target_pose.header.stamp = this->now();
    target_pose.pose.position.x = target_position.x;
    target_pose.pose.position.y = target_position.y;
    // Transform euler angle to quaternion
    tf2::Quaternion quaternion_heading;
    quaternion_heading.setRPY(0, 0, target_position.theta);
    target_pose.pose.orientation.x = quaternion_heading.x();
    target_pose.pose.orientation.y = quaternion_heading.y();
    target_pose.pose.orientation.z = quaternion_heading.z();
    target_pose.pose.orientation.w = quaternion_heading.w();

    // Send target position
    goal_msg.path.poses.push_back(target_pose);

    // Options so results are received, if target was reached
    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SendGoalOptions options;
    // Specify results callback so it is checked if target was reached
    options.result_callback = std::bind(&DwbNavController::ResultCallback, this, std::placeholders::_1);

    // Send the message containing target position asynchronously
    controller_client_->async_send_goal(goal_msg, options);
  }
//------------------------------------------------------------------------------
 private:
  // Functions

  // Odometry callback function
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) const
  {
    current_state.x = msg->pose.pose.position.x;
    current_state.y = msg->pose.pose.position.y;
    current_state.theta = msg->pose.pose.orientation.z;
  }

  // Callback function indicating if target position has been reached
  void ResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult &result)
  {
    switch (result.code) 
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Target position reached.");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "ABORTED path planning to target position.");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "CANCELED path planning to target position.");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
  }

//------------------------------------------------------------------------------
// Variables

  // Odom subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  // Client for sending target position to DWB controller
  rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr controller_client_;
};


// Performs local path planning using DWA.
// Output: N/A
// Input: 
void local_path_planning(RobotState CurrentState, Position TargetPosition)
{
    // Initialize rclcpp in main



    // Shutdown rclcpp in main
};

} // namespace path_planning
} // namesapce robot_controller_interface
