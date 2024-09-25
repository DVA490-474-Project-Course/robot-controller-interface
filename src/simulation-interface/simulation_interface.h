// simulation_interface.h
//==============================================================================
// Author: Emil Åberg
// Creation date: 2024-09-25
// Last modified: 2024-09-25 by Emil Åberg
// Description: Interface that can send UDP data to grSim to make robots move
// and kick
// License: See LICENSE file for license details.
//==============================================================================

#ifndef ROBOTCONTROLLERINTERFACE_SIMULATIONINTERFACE_SIMULATIONINTERFACE_H
#define ROBOTCONTROLLERINTERFACE_SIMULATIONINTERFACE_SIMULATIONINTERFACE_H

// C system headers
#include <arpa/inet.h>

// C++ standard library headers
#include <string>

enum class Team
{
  kBlue = 0,
  kYellow = 1
};

struct VelocityData
{
  int id;
  enum Team team;
  float x_velocity;
  float y_velocity;
  float angular_velocity;
};

class SimulationInterface
{
public:
  SimulationInterface(std::string ip, uint16_t port);
  void SendVelocityData(struct VelocityData velocity_data);
  void ActivateKicker();
private:
  int socket;
  sockaddr_in destination;
};

#endif // ROBOTCONTROLLERINTERFACE_SIMULATIONINTERFACE_SIMULATIONINTERFACE_H