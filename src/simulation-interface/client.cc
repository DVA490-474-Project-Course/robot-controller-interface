#include <sys/socket.h>
#include <netinet/in.h>
#include "client.h"
#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"

Client::Client()
{
    // Define destination (grSim) address
    destination.sin_family = AF_INET;
    destination.sin_port = htons(port);
    destination.sin_addr.s_addr = inet_addr(ip.c_str());

    // Create the client socket
    socket = ::socket(AF_INET, SOCK_DGRAM, 0);
}

Client::~Client()
{

}

void Client::sendPacket()
{
    // Define the robot properties
    grSim_Packet packet;
    packet.mutable_commands()->set_isteamyellow(false);
    packet.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
    command->set_id(1);
    command->set_wheelsspeed(false);
    command->set_wheel1(0);
    command->set_wheel2(0);
    command->set_wheel3(0);
    command->set_wheel4(0);
    command->set_veltangent(1.0);
    command->set_velnormal(0);
    command->set_velangular(0);
    command->set_kickspeedx(0);
    command->set_kickspeedz(0);
    command->set_spinner(false);

    // Serialize the protobuf message before sending
    size_t size = packet.ByteSizeLong();
    void *buffer = (char*)malloc(size);
    packet.SerializeToArray(buffer, size);

    // Send the message with UDP
    ::sendto(socket, buffer, size, 0, reinterpret_cast<sockaddr*>(&destination), sizeof(destination));

    free(buffer);
}
