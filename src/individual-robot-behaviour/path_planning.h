// path_planning.h
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-19
// Last modified: 2024-09-24 by Carl Larsson
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

//==============================================================================

// DWB controller
// Neither copyable nor move-only.
class DwbController : public rclcpp::Node
{
 public:
  DwbController();
  
  // Description: Function for sending the target pose to DWB controller.
  // This should automatically have DWB run in the background and publish
  // velocities on cmd_vel.
  // Use: rclcpp must be initialized before function call
  // Input: Target pose using class Pose
  // Output: N/A
  // Return value: void
  void SendTargetPose(Pose target_pose);

 private:
  // Description: Callback function indicating if target position has been 
  // reached.
  // Use: Use as argument for option in async_send_goal.
  // Input: See bellow 
  // Output: N/A
  // Return value: void
  void ResultCallback(
      const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>
          ::WrappedResult &result);

  // Client for sending target pose to DWB controller
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr 
      navigate_to_pose_client_;
};

//==============================================================================

// Description: Performs local path planning using DWA.
// Use: Function call must be after rclcpp has been initialized. Shutdown rclcpp
// after finished using.
// Input: Target position using class Pose
// Output: N/A
// Return value: void
void local_path_planning(Pose *target_pose);

//==============================================================================

// Global flag used to indicate if target has been reached or not
extern bool target_reached_flag;
// Mutex to protect target_reached_flag
extern std::mutex target_reached_mutex;

//==============================================================================

} // namespace individual_robot_behaviour
} // namesapce robot_controller_interface

#endif // ROBOTCONTROLLERINTERFACE_INDIVIDUALROBOTBEHAVIOUR_PATHPLANNING_H_
