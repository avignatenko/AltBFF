// AlfBFF.cpp : Defines the entry point for the application.
//

#include "AltBFF.h"

#include <BFFCLAPI/UDPClient.h>

#include <Utils/QueueRunner.h>
#include <Utils/Timer.h>

using namespace std;

int main()
{
    QueueRunner runner;

    // fixme: read from settings
    bffcl::UDPClient api("192.168.0.115", 48010, "192.168.0.107", 50001);

    // set timer to stop after some time
    Timer timer(std::chrono::milliseconds(50000), runner, [&runner] {
        runner.stop();
        return true;  // stop
    });

    timer.start();

    float forceIncrement = 2;
    // set timer to update some data once per second
    Timer timerTest(std::chrono::milliseconds(1000), runner, [&api, &forceIncrement] {
        bffcl::CLInput& input = api.lockInput();

        input.elevator.fixedForce = input.elevator.fixedForce + forceIncrement;
        if (std::abs(input.elevator.fixedForce) > 20) forceIncrement *= -1;

        api.unlockInput();

        return false;
    });

    timerTest.start();

    runner.run();

    return 0;
}
