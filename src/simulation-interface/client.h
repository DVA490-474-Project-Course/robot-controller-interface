#ifndef CLIENT_H
#define CLIENT_H

#include <string>
#include <arpa/inet.h>

using namespace std;

class Client
{
public:
    Client();
    ~Client();
public:
    void sendPacket();
private:
    string ip = "127.0.0.1";
    uint16_t port = 20011;
    sockaddr_in destination;
    int socket;
};

#endif // CLIENT_H
