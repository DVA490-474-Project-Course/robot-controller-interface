// path_planning.cc
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-19
// Last modified: 2024-09-22 by Carl Larsson
// Description: Path planning source file, global path planning is not
// necessary, passing the desitnation position instantly and letting DWA (local
// path planning) handle the rest is an acceptable simplification in the 
// SSL-RoboCup environment.
// License: See LICENSE file for license details.
//==============================================================================


// Related .h files
#include "../individual-robot-behaviour/path_planning.h"

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

//==============================================================================

// DWB controller
// Neither copyable nor move-only.
class DwbController : public rclcpp_lifecycle::LifecycleNode
{
 public:
  DwbController() : rclcpp_lifecycle::LifecycleNode("dwb_controller")
  {
    // Define the action client for sending target pose to controller
    navigate_to_pose_client_ = 
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");
  }

  // Description: Function for sending the target pose to DWB controller.
  // This should automatically have DWB run in the background and publish
  // velocities on cmd_vel.
  // Use: rclcpp must be initialized before function call
  // Input: Target pose using class Pose
  // Output: N/A
  // Return value: void
  void SendTargetPose(Pose target_pose) 
  {
    geometry_msgs::msg::PoseStamped target_pose_;
    // Message contents (target pose)
    target_pose_.header.frame_id = "map"; // Change in the future?
    target_pose_.header.stamp = this->now();
    target_pose_.pose.position.x = target_pose.x;
    target_pose_.pose.position.y = target_pose.y;
    // Transform euler angle to quaternion
    tf2::Quaternion quaternion_heading;
    quaternion_heading.setRPY(0, 0, target_pose.theta);
    target_pose_.pose.orientation = tf2::toMsg(quaternion_heading);

    nav2_msgs::action::NavigateToPose::Goal goal_msg;
    // Send target pose
    goal_msg.pose = target_pose_;

    // Options so results are received, if target was reached
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions 
        options;
    // Specify results callback so it is checked if target was reached
    options.result_callback = std::bind(&DwbController::ResultCallback, this, 
        std::placeholders::_1);

    // Send the message containing target pose asynchronously
    navigate_to_pose_client_->async_send_goal(goal_msg, options);
  }
//------------------------------------------------------------------------------
 private:

  // Description: Callback function indicating if target position has been 
  // reached.
  // Use: Use as argument for option in async_send_goal.
  // Input: See bellow 
  // Output: N/A
  // Return value: void
  void ResultCallback(
      const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>
          ::WrappedResult &result)
  {
    switch (result.code) 
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), 
            "Target position reached.");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), 
            "ABORTED path planning to target position.");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), 
            "CANCELED path planning to target position.");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), 
            "Unknown result code");
        break;
    }
  }

  // Client for sending target pose to DWB controller
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr 
      navigate_to_pose_client_;
};

//==============================================================================

// Odom Subscriber
// Neither copyable nor move-only.
class OdomSubscriber : public rclcpp_lifecycle::LifecycleNode 
{
 public:
  OdomSubscriber() : rclcpp_lifecycle::LifecycleNode("odom_subscriber") 
  {
    // Subscribe to odom topic
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&OdomSubscriber::OdomCallback, this, 
            std::placeholders::_1));
  }
 private:
  // Description: Odometry callback function, stores current state in global
  // variable current_state.
  // Use: use as argument when creating odometry subscriber.
  // Input: const shared pointer to msg containing odometry data. 
  // Output N/A
  // Return value: void
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) const
  {
    // Store globally
    current_state.x = msg->pose.pose.position.x;
    current_state.y = msg->pose.pose.position.y;
    current_state.theta = tf2::getYaw(msg->pose.pose.orientation);
  }

  // Odom subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
};

//==============================================================================

// Description: Performs local path planning using DWA.
// Use: Function call must be after rclcpp has been initialized. Shutdown rclcpp
// after finished using.
// Input: Target position using class Pose
// Output: N/A
// Return value: void
void local_path_planning(Pose target_pose)
{
  // Initialize rclcpp in main

  


  // Shutdown rclcpp in main
};

//==============================================================================

} // namespace individual_robot_behaviour
} // namesapce robot_controller_interface
