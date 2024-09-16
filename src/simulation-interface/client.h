#ifndef CLIENT_H
#define CLIENT_H

#include <QUdpSocket>
#include <QHostAddress>
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
public slots:
    void connectUdp();
    void sendPacket();
    void disconnectUdp();
private:
    QUdpSocket udpsocket;
    QHostAddress _addr;
    quint16 _port;
    string ip;
    uint16_t port;

};

#endif // CLIENT_H
