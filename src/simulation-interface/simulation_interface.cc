/* simulation_interface.cc
 *==============================================================================
 * Author: Emil Åberg, Shruthi Puthiya Kunnon, Aaiza Aziz Khan
 * Creation date: 2024-09-25
 * Last modified: 2024-10-07 by Emil Åberg
 * Description: Interface that can send UDP data to grSim to make robots move
 * and kick
 * License: See LICENSE file for license details.
 *==============================================================================
 */

/* Related .h files */
#include "simulation_interface.h"

/* C system headers */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

/* C++ standard library headers */
#include <memory>
#include <string>

/* Project .h files */
#include "grSim_Commands.pb.h"
#include "grSim_Packet.pb.h"
#include "../common_types.h"

namespace robot_controller_interface
{
namespace simulation_interface
{

/* Constructor */
SimulationInterface::SimulationInterface(std::string ip, uint16_t port)
{
  /* Define destination address */
  destination.sin_family = AF_INET;
  destination.sin_port = htons(port);
  destination.sin_addr.s_addr = inet_addr(ip.c_str());

  /* Create the client socket */
  socket = ::socket(AF_INET, SOCK_DGRAM, 0);
}

/* Send a UDP packet with the robot command */
void SimulationInterface::SendRobotCommand(
  int id, enum Team team, bool spinner_on, float kicker_speed,
  float x_speed, float y_speed, float angular_speed)
{
  grSim_Packet packet;
  grSim_Robot_Command *command;

  /* Write the data to the protobuf message */
  packet.mutable_commands()->set_isteamyellow(team == Team::kYellow);
  packet.mutable_commands()->set_timestamp(0.0L);
  command = packet.mutable_commands()->add_robot_commands();
  command->set_id(id);
  command->set_wheelsspeed(false);
  command->set_veltangent(x_speed);
  command->set_velnormal(y_speed);
  command->set_velangular(angular_speed);
  command->set_kickspeedx(kicker_speed);
  command->set_kickspeedz(0.0F);
  command->set_spinner(spinner_on);

  /* Send the packet */
  SendPacket(packet);
}

/* Send a UDP packet with the robot command */
void SimulationInterface::SendRobotCommand(
  int id, enum Team team, bool spinner_on, float kicker_speed,
  float wheel1, float wheel2,float wheel3, float wheel4)
{
  grSim_Packet packet;
  grSim_Robot_Command *command;

  /* Write the data to the protobuf message */
  packet.mutable_commands()->set_isteamyellow(team == Team::kYellow);
  packet.mutable_commands()->set_timestamp(0.0L);
  command = packet.mutable_commands()->add_robot_commands();
  command->set_id(id);
  command->set_wheelsspeed(true);
  command->set_wheel1(wheel1);
  command->set_wheel1(wheel2);
  command->set_wheel1(wheel3);
  command->set_wheel1(wheel4);
  command->set_kickspeedz(0.0F);
  command->set_spinner(spinner_on);

  /* Send the packet */
  SendPacket(packet);
}

void SimulationInterface::ResetRobotsAndBall()
{
  grSim_Packet packet;
  grSim_Robot_Command *command;
  grSim_RobotReplacement *replacement;
  grSim_BallReplacement *ball_replacement;

  packet.mutable_commands()->set_isteamyellow(false); /* Set to false for blue team */
  packet.mutable_commands()->set_timestamp(0.0L);

  /* Loop through each robot index to reset the positions and other attributes
  of blue and yellow team*/
  for (int k = 0; k < 6; k++)
  {
    /* Reset blue team robots (yellowteam = false) */
    command = packet.mutable_commands()->add_robot_commands();
    command->set_id(k);
    command->set_wheelsspeed(false);
    command->set_veltangent(0.0F); // Stop all movement
    command->set_velnormal(0.0F);  // Stop all movement
    command->set_velangular(0.0F); // Stop angular movement
    command->set_kickspeedx(0.0F); // No kick speed
    command->set_kickspeedz(0.0F); // No kick in Z direction
    command->set_spinner(false);   // Turn off the spinner

    /* Set up the replacement packet for blue team */
    replacement = packet.mutable_replacement()->add_robots();
    replacement->set_id(k);
    replacement->set_x(-initial_position_x[k]);          // Set new x position
    replacement->set_y(initial_position_y[k]);           // Set new y position
    replacement->set_dir(0.0F);         // Set direction (angle in radians)
    replacement->set_yellowteam(false); // Set to blue team (yellowteam = false)

    /* Reset yellow team robots (yellowteam = true) */
    command = packet.mutable_commands()->add_robot_commands();
    command->set_id(k);
    command->set_wheelsspeed(false);
    command->set_veltangent(0.0F); // Stop all movement
    command->set_velnormal(0.0F);  // Stop all movement
    command->set_velangular(0.0F); // Stop angular movement
    command->set_kickspeedx(0.0F); // No kick speed
    command->set_kickspeedz(0.0F); // No kick in Z direction
    command->set_spinner(false);   // Turn off the spinner

    /* Set up the replacement packet for yellow team */
    replacement = packet.mutable_replacement()->add_robots();
    replacement->set_id(k);
    replacement->set_x(initial_position_x[k]);
    replacement->set_y(initial_position_y[k]);
    replacement->set_dir(0.0F);
    replacement->set_yellowteam(true);
  }

  /* Replacement packet for ball */
  ball_replacement = packet.mutable_replacement()->mutable_ball();
  ball_replacement->set_x(0.0F);
  ball_replacement->set_y(0.0F);
  ball_replacement->set_vx(0.0F);
  ball_replacement->set_vy(0.0F);

  /* Send the packet */
  SendPacket(packet);
}

/* Send a grSim packet with UDP */
void SimulationInterface::SendPacket(grSim_Packet packet)
{
  size_t size;
  void *buffer;

  /* Serialize the protobuf message before sending */
  size = packet.ByteSizeLong();
  buffer = (char *)malloc(size);
  packet.SerializeToArray(buffer, size);

  /* Send the UDP packet*/
  ::sendto(socket, buffer, size, 0, reinterpret_cast<sockaddr *>(&destination),
           sizeof(destination));

  free(buffer);
}

} /* namespace simulation_interface */
} /* namespace robot_controller_interface */