#include <QByteArray>
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include "client.h"

Client::Client()
{

}

Client::~Client()
{
    ip = "127.0.0.1";
    port = 20011;
}

void Client::disconnectUdp()
{
    //udpsocket.close();
}

void Client::reconnectUdp()
{
    //_addr = QHostAddress(ip);
    //_port = port.toUShort();
}

void Client::sendPacket()
{
    grSim_Packet packet;
    packet.mutable_commands()->set_isteamyellow(false);
    packet.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();

    command->set_id(0);
    command->set_wheelsspeed(10);
    command->set_wheel1(0);
    command->set_wheel2(0);
    command->set_wheel3(0);
    command->set_wheel4(0);
    command->set_veltangent(0);
    command->set_velnormal(0);
    command->set_velangular(0);
    command->set_kickspeedx(0);
    command->set_kickspeedz(0);
    command->set_spinner(false);

    //QByteArray dgram;
    //dgram.resize(packet.ByteSize());
    //packet.SerializeToArray(dgram.data(), dgram.size());
    //udpsocket.writeDatagram(dgram, _addr, _port);

    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in destination;
    destination.sin_family = AF_INET;
    destination.sin_port = htons(port);
    destination.sin_addr.s_addr = inet_addr(ip.c_str());

    string msg = "Jane Doe"; // for test
    ::sendto(sock, msg.c_str(), msg.length(), 0, reinterpret_cast<sockaddr*>(&destination), sizeof(destination));
    ::close(sock);
}
