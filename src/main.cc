#include <iostream>
#include <fstream>
#include <string>
#include "client.h"

int main(int argc, char *argv[])
{
    Client client;
    client.reconnectUdp();
    client.sendPacket();
    client.disconnectUdp();
    return 0;
}
