#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include "client.h"

int main(int argc, char *argv[])
{
    Client client;
    while (true)
    {
        client.sendPacket();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
}
