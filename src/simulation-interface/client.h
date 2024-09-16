#ifndef CLIENT_H
#define CLIENT_H

#include <sys/types.h>
//#include "grSim_Packet.pb.h"
//#include "grSim_Commands.pb.h"
//#include "grSim_Replacement.pb.h"

using namespace std;

class Client
{
public:
    Client();
    ~Client();
public:
    void connectUdp();
    void sendPacket();
    void disconnectUdp();
private:
    string ip;
    uint16_t port;

};

#endif // CLIENT_H
