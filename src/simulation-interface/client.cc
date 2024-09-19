// client.cc
//==============================================================================
// Author: Emil Åberg
// Creation date: 2024-09-16
// Last modified: 2024-09-19 by Emil Åberg
// Description: A simple UDP client that for now only can send a packet to move
// one robot in grSim
// License: See LICENSE file for license details.
//==============================================================================

#include "client.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <string>
#include "grSim_Commands.pb.h"
#include "grSim_Packet.pb.h"
#include "grSim_Replacement.pb.h"

// Constructor
Client::Client()
{
    // Define destination (grSim) address
    destination.sin_family = AF_INET;
    destination.sin_port = htons(port);
    destination.sin_addr.s_addr = inet_addr(ip.c_str());

    // Create the client socket
    socket = ::socket(AF_INET, SOCK_DGRAM, 0);
}

// Destructor
Client::~Client()
{

}

// Send one UDP packet
void Client::SendPacket()
{
    grSim_Packet packet;
    size_t size;
    void *buffer;
    grSim_Robot_Command* command;

    // Define the robot properties
    packet.mutable_commands()->set_isteamyellow(false);
    packet.mutable_commands()->set_timestamp(0.0L);
    command = packet.mutable_commands()->add_robot_commands();
    command->set_id(1);
    command->set_wheelsspeed(false);
    command->set_wheel1(0.0L);
    command->set_wheel2(0.0L);
    command->set_wheel3(0.0L);
    command->set_wheel4(0.0L);
    command->set_veltangent(1.0L);
    command->set_velnormal(0.0L);
    command->set_velangular(0.0L);
    command->set_kickspeedx(0.0L);
    command->set_kickspeedz(0.0L);
    command->set_spinner(false);

    // Serialize the protobuf message before sending
    size = packet.ByteSizeLong();
    buffer = (char*)malloc(size);
    packet.SerializeToArray(buffer, size);

    // Send the message with UDP
    ::sendto(socket, buffer, size, 0, reinterpret_cast<sockaddr*>(&destination), sizeof(destination));

    free(buffer);
}
