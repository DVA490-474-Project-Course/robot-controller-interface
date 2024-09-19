// client.h
//==============================================================================
// Author: Emil Åberg
// Creation date: 2024-09-16
// Last modified: 2024-09-19 by Emil Åberg
// Description: A simple UDP client that for now only can send a packet to move
// one robot in grSim
// License: See LICENSE file for license details.
//==============================================================================

#ifndef ROBOTCONTROLLERINTERFACE_SIMULATIONINTERFACE_CLIENT_H
#define ROBOTCONTROLLERINTERFACE_SIMULATIONINTERFACE_CLIENT_H

#include <string>
#include <arpa/inet.h>

class Client
{
public:
    Client();
    ~Client();
    void SendPacket();
private:
    std::string ip = "127.0.0.1";
    uint16_t port = 20011;
    sockaddr_in destination;
    int socket;
};

#endif // ROBOTCONTROLLERINTERFACE_SIMULATIONINTERFACE_CLIENT_H
