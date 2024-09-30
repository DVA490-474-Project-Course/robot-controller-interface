// path_planning.cc
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-19
// Last modified: 2024-09-24 by Carl Larsson
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
#include <mutex>

// Other .h files
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_controller/controller_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"

// Project .h files
#include "../individual-robot-behaviour/ball.h"
#include "../individual-robot-behaviour/state.h"


namespace robot_controller_interface
{
namespace individual_robot_behaviour
{

//==============================================================================

// Used to indicate if work is completed. 
// 0 is false
// 1 then work called by local path planner is completed
// 2 then work called by shoot setup is completed
std::atomic_int atomic_target_reached_flag = 0;
// Used to keep track of who has asked path planning to do work
std::atomic_bool atomic_move_to_target = false;
// Global mutex to protect target_pose (pointer)
std::mutex target_pose_mutex;

//==============================================================================

// DWB controller
// Neither copyable nor move-only.
DwbController::DwbController() : rclcpp::Node("dwb_controller")
{
  // Define the action client for sending target pose to controller
  navigate_to_pose_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, "navigate_to_pose");

  // Ensure the action server is available
  if (!navigate_to_pose_client_->wait_for_action_server(std::chrono
        ::seconds(10))) 
  {
    RCLCPP_ERROR(this->get_logger(), "Action server not available!");
  }
}

// Description: Function for sending the target pose to DWB controller.
// This should automatically have DWB run in the background and publish
// velocities on cmd_vel.
// Use: rclcpp must be initialized before function call
// Input: Target pose using class Pose
// Output: N/A
// Return value: void
void DwbController::SendTargetPose(Pose target_pose)
{
  geometry_msgs::msg::PoseStamped target_pose_;
  // Message contents (target pose)
  target_pose_.header.frame_id = "map"; // Change in the future?
  target_pose_.header.stamp = this->now();
  target_pose_.pose.position.x = target_pose.GetX();
  target_pose_.pose.position.y = target_pose.GetY();
  
  // Transform Euler angle to quaternion
  tf2::Quaternion quaternion_heading;
  quaternion_heading.setRPY(0, 0, target_pose.GetTheta());
  target_pose_.pose.orientation = tf2::toMsg(quaternion_heading);

  nav2_msgs::action::NavigateToPose::Goal goal_msg;
  // Send target pose
  goal_msg.pose = target_pose_;

  // Options to receive results if target was reached
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions 
      options;
  // Specify results callback
  options.result_callback = std::bind(&DwbController::ResultCallback, this, 
      std::placeholders::_1);

  // Send the message containing target pose asynchronously
  navigate_to_pose_client_->async_send_goal(goal_msg, options);
}

// Description: Callback function indicating if target position has been 
// reached.
// Use: Use as argument for option in async_send_goal.
// Input: See bellow 
// Output: N/A
// Return value: void
void DwbController::ResultCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>
        ::WrappedResult &result)
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Target position reached.");
      // Thread safe
      // If local path planning called, indicate to it that target has been
      // reached
      if(atomic_move_to_target)
      {
        atomic_target_reached_flag = 1;
        atomic_move_to_target = false;
      }
      // If shoot_setup called, indicate that task is complete and correct
      // direction has been obtained
      else if(atomic_shoot_setup_work) 
      {
        atomic_target_reached_flag = 2;
        atomic_shoot_setup_work = false;
      }
      // Something is weird if we get here
      else 
      {
        atomic_target_reached_flag = 0;
        atomic_move_to_target = false;
        atomic_shoot_setup_work = false;
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), 
          "ABORTED path planning to target position.");
      atomic_target_reached_flag = 0;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), 
          "CANCELED path planning to target position.");
      atomic_target_reached_flag = 0;
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      atomic_target_reached_flag = 0;
      break;
  }
}

//==============================================================================

// Description: Performs local path planning using DWA.
// Use: Function call must be after rclcpp has been initialized. Shutdown rclcpp
// after finished using.
// Input: Target position using class Pose
// Output: N/A
// Return value: void
void local_path_planning(Pose *target_pose)
{
  // Initialize rclcpp in main

  // Local copy of most recent target pose, used to check if new target has 
  // been set.
  // Make it thread safe
  target_pose_mutex.lock();
  Pose current_target = *target_pose;
  target_pose_mutex.unlock();
  
  // Create DWB node
  DwbController dwb_node;
  // Pointer to DWB node
  std::shared_ptr<DwbController> dwb_node_ptr = std::make_shared<DwbController>();
  // Send target pose
  dwb_node.SendTargetPose(current_target);

  // Set the execution rate to 10 Hz for this loop
  rclcpp::Rate rate(10);

  // Loop forever
  while(rclcpp::ok())
  {
    // Do work for a while
    rclcpp::spin_some(dwb_node_ptr->get_node_base_interface());

    // Make it thread safe
    target_pose_mutex.lock();
    // We have received a new target
    if(*target_pose != current_target)
    {
      // Store new target in local copy
      current_target.SetX((*target_pose).GetX());
      current_target.SetY((*target_pose).GetY());
      current_target.SetTheta((*target_pose).GetTheta());
      // Send new target pose
      // This will cancel the old one and retarget to the new one
      dwb_node.SendTargetPose(current_target);

      // Reset flag
      atomic_target_reached_flag = 0;
    }
    target_pose_mutex.unlock();

    // Ensure 10 Hz is maintained
    rate.sleep();
  }

  // Shutdown rclcpp in main
};

//==============================================================================

} // namespace individual_robot_behaviour
} // namesapce robot_controller_interface
