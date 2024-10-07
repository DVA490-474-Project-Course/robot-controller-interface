// simulation_interface.cc
//==============================================================================
// Author: Emil Åberg, Shruthi Puthiya Kunnon, Aaiza Aziz Khan
// Creation date: 2024-09-25
// Last modified: 2024-10-07 by Shruthi and Aaiza
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
#include <memory>
#include <string>

// Project .h files
#include "grSim_Commands.pb.h"
#include "grSim_Packet.pb.h"

namespace robot_controller_interface {
namespace simulation_interface {

// Constructor
SimulationInterface::SimulationInterface(std::string ip, uint16_t port) {
  // Define destination address
  destination.sin_family = AF_INET;
  destination.sin_port = htons(port);
  destination.sin_addr.s_addr = inet_addr(ip.c_str());

  // Create the client socket
  socket = ::socket(AF_INET, SOCK_DGRAM, 0);
}

// Send a UDP packet
void SimulationInterface::SendRobotData(struct RobotData robot_data) {
  grSim_Packet packet;
  grSim_Robot_Command *command;
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
  buffer = (char *)malloc(size);
  packet.SerializeToArray(buffer, size);

  // Send the message with UDP
  ::sendto(socket, buffer, size, 0, reinterpret_cast<sockaddr *>(&destination),
           sizeof(destination));

  free(buffer);
}

void SimulationInterface::ResetRobotsAndBall(struct RobotData robot_data) {
  grSim_Packet packet;
  grSim_Robot_Command *command;
  grSim_RobotReplacement *replacement;
  grSim_BallReplacement *ball_replacement;
  size_t size;
  void *buffer;

  packet.mutable_commands()->set_isteamyellow(false); // Set to false for blue team
  packet.mutable_commands()->set_timestamp(0.0L);

  //initial position of yellow robots for blue x values will have opposite sign
  double x[6] = {1.50, 1.50, 1.50, 0.55, 2.50, 3.60};
  double y[6] = {1.12, 0.0, -1.12, 0.00, 0.00, 0.00};

  /* Loop through each robot index to reset the positions and other attributes
  of blue and yellow team*/
  for (int k = 0; k < 6; k++) {
    // Reset blue team robots (yellowteam = false)
    command = packet.mutable_commands()->add_robot_commands();
    command->set_id(k);
    command->set_wheelsspeed(false);
    command->set_veltangent(0.0F); // Stop all movement
    command->set_velnormal(0.0F);  // Stop all movement
    command->set_velangular(0.0F); // Stop angular movement
    command->set_kickspeedx(0.0F); // No kick speed
    command->set_kickspeedz(0.0F); // No kick in Z direction
    command->set_spinner(false);   // Turn off the spinner

    // Set up the replacement packet for blue team
    replacement = packet.mutable_replacement()->add_robots();
    replacement->set_id(k);
    replacement->set_x(-x[k]);          // Set new x position
    replacement->set_y(y[k]);           // Set new y position
    replacement->set_dir(0.0F);         // Set direction (angle in radians)
    replacement->set_yellowteam(false); // Set to blue team (yellowteam = false)

    // Reset yellow team robots (yellowteam = true)
    command = packet.mutable_commands()->add_robot_commands();
    command->set_id(k);
    command->set_wheelsspeed(false);
    command->set_veltangent(0.0F); // Stop all movement
    command->set_velnormal(0.0F);  // Stop all movement
    command->set_velangular(0.0F); // Stop angular movement
    command->set_kickspeedx(0.0F); // No kick speed
    command->set_kickspeedz(0.0F); // No kick in Z direction
    command->set_spinner(false);   // Turn off the spinner

    // Set up the replacement packet for yellow team
    replacement = packet.mutable_replacement()->add_robots();
    replacement->set_id(k);
    replacement->set_x(x[k]);
    replacement->set_y(y[k]);
    replacement->set_dir(0.0F);
    replacement->set_yellowteam(true);
  }

  //Replacement packet for ball
  ball_replacement = packet.mutable_replacement()->mutable_ball();
  ball_replacement->set_x(0.0);
  ball_replacement->set_y(0.0);
  ball_replacement->set_vx(0.0);
  ball_replacement->set_vy(0.0);

  size = packet.ByteSizeLong();
  buffer = (char *)malloc(size);
  packet.SerializeToArray(buffer, size);

  // Send the message with UDP
  ::sendto(socket, buffer, size, 0, reinterpret_cast<sockaddr *>(&destination),
           sizeof(destination));

  free(buffer);

} // reset fuction ends
} // namespace simulation_interface
} // namespace robot_controller_interface