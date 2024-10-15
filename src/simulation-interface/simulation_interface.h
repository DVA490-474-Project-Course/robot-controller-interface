/* simulation_interface.h
 *==============================================================================
 * Author: Emil Åberg, Shruthi Puthiya Kunnon, Aaiza Aziz Khan
 * Creation date: 2024-09-25
 * Last modified: 2024-10-07 by Emil Åberg
 * Description: Interface that can send UDP data to grSim to make robots move
 * and kick
 * License: See LICENSE file for license details.
 *==============================================================================
 */

#ifndef ROBOTCONTROLLERINTERFACE_SIMULATIONINTERFACE_SIMULATIONINTERFACE_H
#define ROBOTCONTROLLERINTERFACE_SIMULATIONINTERFACE_SIMULATIONINTERFACE_H

/* C system headers */
#include <arpa/inet.h>

/* C++ standard library headers */
#include <string>

/* Project .h files */
#include "grSim_Packet.pb.h"
#include "../common_types.h"

namespace robot_controller_interface
{
namespace simulation_interface
{

class SimulationInterface
{
public:
  /* Constructor */
  SimulationInterface(std::string ip, uint16_t port);

  /* Functions to send robot command to grSim */
  void SendRobotCommand(int id, enum Team team, bool spinner_on, float kicker_speed,
    float x_speed, float y_speed, float angular_speed);
  void SendRobotCommand(int id, enum Team team, bool spinner_on, float kicker_speed,
    float wheel1, float wheel2,float wheel3, float wheel4);

  /* Reset ball and all robots position and other attributes */
  void ResetRobotsAndBall();
private:
  /* Network variables */
  int socket;
  sockaddr_in destination;

  /* initial position of yellow robots, for blue x values will have opposite sign */
  double initial_position_x[6] = {1.50, 1.50, 1.50, 0.55, 2.50, 3.60};
  double initial_position_y[6] = {1.12, 0.0, -1.12, 0.00, 0.00, 0.00};

  /* Helper functions */
  void SendPacket(grSim_Packet packet);
};

} /* namespace simulation_interface */
} /* namesapce robot_controller_interface */

#endif /* ROBOTCONTROLLERINTERFACE_SIMULATIONINTERFACE_SIMULATIONINTERFACE_H */