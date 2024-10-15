/* simulation_interface_demo.cc
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-09-16
 * Last modified: 2024-10-15 by Emil Åberg
 * Description: Demo of the simulation interface
 * License: See LICENSE file for license details.
 *==============================================================================
 */

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include "simulation_interface.h"

int main(int argc, char *argv[])
{
  robot_controller_interface::simulation_interface::SimulationInterface simulation_interface(
    "127.0.0.1", 20011);

  /* Run a loop to send commands and then reset the robot after 600 iterations */
  int count = 0;
  while (true)
  {
    simulation_interface.SendRobotCommand(
      3, robot_controller_interface::Team::kBlue, false, 10.0F, 0.0F, 0.0F, 0.0F);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    /* Reset blue and yellow team robots and ball every 6 seconds */
    if (count >= 600)
    {
      std::cout << "Resetting robots and ball..." << std::endl;
      simulation_interface.ResetRobotsAndBall();
      count = 0; /* Reset the counter after the reset */
      std::this_thread::sleep_for(std::chrono::seconds(10)); /* Pause for a short period after reset */
    }

    count++;
  }

  return 0;
}