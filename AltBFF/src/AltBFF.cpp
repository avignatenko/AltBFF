// AlfBFF.cpp : Defines the entry point for the application.
//

#include "AltBFF.h"

#include <BFFCLAPI/UDPClient.h>

#include <Utils/QueueRunner.h>
#include <Utils/Timer.h>

using namespace std;

int main()
{
    bffcl::UDPClient api("192.168.0.115", 50000, 50001);

    QueueRunner runner;

    // set timer to stop after some time
    Timer timer(std::chrono::milliseconds(5000), runner, [&runner] { runner.stop(); });
    timer.singleShot();

    runner.run();

    return 0;
}
