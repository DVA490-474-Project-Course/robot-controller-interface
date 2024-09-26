// main.cc
//==============================================================================
// Author: Carl Larsson, Emil Åberg
// Creation date: 2024-09-16
// Last modified: 2024-09-26 by Carl Larsson
// Description: Main, the executable instance for one robot. Does 
// initialization, path thread planning, shooting setup and calling shoot 
// thread, centralized AI listener thread, and finally shutdown.
// License: See LICENSE file for license details.
//==============================================================================


// Related .h files

// C++ standard library headers
#include <thread>

// Other .h files
#include "rclcpp/rclcpp.hpp"

// Project .h files
#include "./individual-robot-behaviour/path_planning.h"
#include "./individual-robot-behaviour/state.h"
#include "./individual-robot-behaviour/ball.h"
#include "./individual-robot-behaviour/supporting.h"


//==============================================================================

int main(int argc, char **argv)
{
  // Variables
  robot_controller_interface::individual_robot_behaviour::Pose *target_position;
  int *robot_id;

  // Start rclcpp
  rclcpp::init(argc, argv);

  // Initialize robot
  // Gain robot ID and initial pose
  robot_controller_interface::individual_robot_behaviour::initialize_robot(robot_id, target_position);

  /* TODO Complete the functions the threads are to run */
  /*
  // Fix threads
  // Does path planning
  std::thread path_planning_thread(local_path_planning, target_position);
  // Handles setting the robot up for a shot/kick, then calls function
  // which activates kicker
  std::thread ball_thread(shoot_setup, );
  // Listens to the centralized computer 
  // (AI commands, correction data)
  std::thread listener_thread(listener, target_position, );
  // Sends data to centralized computer
  // (Sensor data, robot status information)
  std::thread sender_thread(sender, );
  */

  // Shutdown rclcpp
  rclcpp::shutdown();

  return 0;
}

//==============================================================================
