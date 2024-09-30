// simulation_interface.cc
//==============================================================================
// Author: Emil Åberg
// Creation date: 2024-09-25
// Last modified: 2024-09-25 by Emil Åberg
// Description: Interface that can send UDP data to grSim to make robots move
// and kick
// License: See LICENSE file for license details.
//==============================================================================

// Related .h files
#include "simulation_interface.h"

// C system headers
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

// C++ standard library headers
#include <string>
#include <memory>

// Project .h files
#include "grSim_Commands.pb.h"
#include "grSim_Packet.pb.h"

// Constructor
SimulationInterface::SimulationInterface(std::string ip, uint16_t port)
{
  // Define destination address'
  destination.sin_family = AF_INET;
  destination.sin_port = htons(port);
  destination.sin_addr.s_addr = inet_addr(ip.c_str());

  // Create the client socket
  socket = ::socket(AF_INET, SOCK_DGRAM, 0);
}

// Send a UDP packet for each robot
void SimulationInterface::SendRobotData(struct RobotData robot_data)
{
  grSim_Packet packet;
  grSim_Robot_Command* command;
  size_t size;
  void *buffer;

  // Write the data to the protobuf message
  packet.mutable_commands()->set_isteamyellow(robot_data.team == Team::kYellow);
  packet.mutable_commands()->set_timestamp(0.0L);
  command = packet.mutable_commands()->add_robot_commands();
  command->set_id(robot_data.id);
  command->set_wheelsspeed(false);
  command->set_veltangent(robot_data.x_velocity);
  command->set_velnormal(robot_data.y_velocity);
  command->set_velangular(robot_data.angular_velocity);
  command->set_kickspeedx(robot_data.kick_speed);
  command->set_kickspeedz(0.0F);
  command->set_spinner(robot_data.spinner_on);

  // Serialize the protobuf message before sending
  size = packet.ByteSizeLong();
  buffer = (char*)malloc(size);
  packet.SerializeToArray(buffer, size);

  // Send the message with UDP
  ::sendto(socket, buffer, size, 0, reinterpret_cast<sockaddr*>(&destination), sizeof(destination));

  free(buffer);
}