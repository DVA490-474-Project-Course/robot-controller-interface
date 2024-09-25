// main.cc
//==============================================================================
// Author: Emil Åberg
// Creation date: 2024-09-16
// Last modified: 2024-09-19 by Emil Åberg
// Description: Main
// License: See LICENSE file for license details.
//==============================================================================

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include "simulation_interface.h"

int main(int argc, char *argv[])
{
  // example data to send
  struct VelocityData velocity_data =
  {
    .id = 1,
    .team = Team::kBlue,
    .x_velocity = 10.0F,
    .y_velocity = 0.0F,
    .angular_velocity = 0.0F
  };

  // demo the simulation interface
  SimulationInterface simulation_interface("127.0.0.1", 20011);
  while (true)
  {
    simulation_interface.SendVelocityData(velocity_data);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  return 0;
}
