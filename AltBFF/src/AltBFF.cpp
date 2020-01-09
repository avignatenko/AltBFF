// AlfBFF.cpp : Defines the entry point for the application.
//

#include "AltBFF.h"

#include <BFFCLAPI/UDPClient.h>

#include <Utils/QueueRunner.h>
#include <Utils/Timer.h>

using namespace std;

#include <stdio.h>
#include <windows.h>

BOOL WINAPI CtrlHandler(DWORD fdwCtrlType)
{
    // todo: add closing

    return FALSE;  // let others work on this
}

#include <windows.h>
#include "FSUIPC_User64.h"

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <filesystem>

void initLogging()
{
    // init logging

    // Set the default logger to file logger
    spdlog::set_level(spdlog::level::info);  // Set global log level to debug

    // auto logFilename = exePath / "altfs.log";
    // auto file_logger = spdlog::basic_logger_mt("basic_logger", logFilename.string());
    // spdlog::set_default_logger(file_logger);
    auto console = spdlog::stdout_color_mt("console");
    spdlog::set_default_logger(console);

    spdlog::info("AltBFF started");
}

int main(int argc, char** argv)
{
    initLogging();

    DWORD dwResult;

    bool fsuipcPresent = FSUIPC_Open(SIM_ANY, &dwResult);
    if (fsuipcPresent)
        spdlog::info("FSUIPC found");
    else
        spdlog::error("FSUIPC not found (error {}), continue in test mode", dwResult);

    SetConsoleCtrlHandler(CtrlHandler, TRUE);

    std::filesystem::path exeName = argv[0];
    std::filesystem::path exePath = exeName.parent_path();

    boost::property_tree::ptree settings;

    auto settingsFile = (exePath / "settings.ini").string();
    spdlog::info("Reading settings file {}", settingsFile);
    boost::property_tree::read_ini(settingsFile, settings);
    spdlog::info("Settings file read successfully");

    auto port = settings.get<std::string>("Network.CLIPAddress");

    bffcl::UDPClient api(settings.get<std::string>("Network.CLIPAddress"), settings.get<int>("Network.CLPort"),
                         settings.get<std::string>("Network.ThisIPAddress"), settings.get<int>("Network.ThisPort"));

    // set values not changing during simulation
    auto& input = api.lockInput();
    input.aileron.frictionCoeff = settings.get<int>("Model.AileronFrictionCoeff");
    input.aileron.dumpingCoeff = settings.get<int>("Model.AileronDumpingCoeff");
    input.elevator.frictionCoeff = settings.get<int>("Model.ElevatorFrictionCoeff");
    input.elevator.dumpingCoeff = settings.get<int>("Model.ElevatorDumpingCoeff");
    api.unlockInput();

    QueueRunner runner;

    /*
    // set timer to stop after some time
    Timer timer(std::chrono::milliseconds(50000), runner, [&runner] {
        runner.stop();
        return true;  // stop
    });

    timer.start(); */

    /*
    float forceIncrement = 2;
    // set timer to update some data once per second
    Timer timerTest(std::chrono::milliseconds(1000), runner, [&api, &forceIncrement] {
        bffcl::CLInput& input = api.lockInput();

        input.elevator.fixedForce = input.elevator.fixedForce + forceIncrement;
        if (std::abs(input.elevator.fixedForce) > 20) forceIncrement *= -1;

        api.unlockInput();

        return false;
    });

    timerTest.start();*/

    /*
    Timer timerReadBack(std::chrono::milliseconds(1000 / 30), runner, [&api] {
        const bffcl::CLReturn& output = api.lockOutput();

        std::cout << output.axisElevatorPosition << std::endl;

        api.unlockOutput();

        return false;
    });*/

    struct Status
    {
        int16_t elevatorPosition = 0;
        int16_t aileronPosition = 0;
    };

    struct Settings
    {
        bool invertFSElevator = false;
        bool invertFSAileron = false;
    };

    Status status;
    Settings simSettings;
    simSettings.invertFSElevator = settings.get<bool>("Sim.InvertElevator");
    simSettings.invertFSAileron = settings.get<bool>("Sim.InvertAileron");

    Timer FSUIPC(std::chrono::milliseconds(1000 / 30), runner, [&api, &status, &simSettings] {
        const bffcl::CLReturn& output = api.lockOutput();
        status.elevatorPosition =
            static_cast<int16_t>(output.axisElevatorPosition * 16383 / 100) * (simSettings.invertFSElevator ? -1 : 1);
        status.aileronPosition =
            static_cast<int16_t>(output.axisAileronPosition * 16383 / 100) * (simSettings.invertFSAileron ? -1 : 1);
        api.unlockOutput();

        DWORD dwResult;
        FSUIPC_Write(0x0BB2, 2, &status.elevatorPosition, &dwResult);
        FSUIPC_Write(0x0BB6, 2, &status.aileronPosition, &dwResult);
        FSUIPC_Process(&dwResult);

        return false;
    });

    if (fsuipcPresent)
    {
        spdlog::info("Starting FSUIPC loop");
        FSUIPC.start();
        spdlog::info("FSUIPC loop started");
    }

    spdlog::info("Starting main loop");
    runner.run();
    spdlog::info("Main loop finished");

    return 0;
}
