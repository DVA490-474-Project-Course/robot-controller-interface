// main.cc
//==============================================================================
// Author: Emil Åberg
// Creation date: 2024-09-16
// Last modified: 2024-09-19 by Emil Åberg
// Description: Main
// License: See LICENSE file for license details.
//==============================================================================

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include "client.h"

int main(int argc, char *argv[])
{
    // Demo the client
    Client client;
    while (true)
    {
        client.SendPacket();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
}
