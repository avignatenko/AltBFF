
#include <BFFCLAPI/UDPClient.h>
#include "Model.h"
#include "Sim.h"

#include <Utils/QueueRunner.h>
#include <Utils/Timer.h>

#include <stdio.h>
#include <string>
#include <windows.h>

using namespace std;


BOOL WINAPI CtrlHandler(DWORD fdwCtrlType)
{
    // todo: add closing

    return FALSE;  // let others work on this
}

#include <windows.h>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <boost/math/constants/constants.hpp>
#include <filesystem>

using boost::property_tree::ptree;
using std::filesystem::path;

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

ptree readSettings(const path& file)
{
    ptree settings;

    auto settingsFile = (file / "settings.ini").string();
    spdlog::info("Reading settings file {}", settingsFile);
    boost::property_tree::read_ini(settingsFile, settings);
    spdlog::info("Settings file read successfully");

    return settings;
}

bffcl::UDPClient::Settings readCLSettings(const ptree& settings)
{
    bffcl::UDPClient::Settings clSettings;
    clSettings.toAddress = settings.get<std::string>("Network.CLIPAddress");
    clSettings.toPort = settings.get<int>("Network.CLPort"),
    clSettings.fromAddress = settings.get<std::string>("Network.ThisIPAddress"),
    clSettings.fromPort = settings.get<int>("Network.ThisPort");

    return clSettings;
}

Sim::Settings readSimSettings(const ptree& settings)
{
    Sim::Settings simSettings;
    simSettings.invertFSElevator = settings.get<bool>("Sim.InvertElevator");
    simSettings.invertFSAileron = settings.get<bool>("Sim.InvertAileron");
    simSettings.clElevatorTrimOffset =
        std::stoul(settings.get<std::string>("Sim.CLElevatorTrimOffset"), nullptr, 16);
    return simSettings;
}

Model::Settings readModelSettings(const ptree& settings)
{
    Model::Settings modelSettings;
    modelSettings.aileronFrictionCoeff = settings.get<int>("Model.AileronFrictionCoeff");
    modelSettings.aileronDumpingCoeff = settings.get<int>("Model.AileronDumpingCoeff");
    modelSettings.elevatorFrictionCoeff = settings.get<int>("Model.ElevatorFrictionCoeff");
    modelSettings.elevatorDumpingCoeff = settings.get<int>("Model.ElevatorDumpingCoeff");

    modelSettings.elevatorArea = settings.get<double>("Model.ElevatorArea");
    modelSettings.clExponent = settings.get<double>("Model.CLExponent");
    modelSettings.propWashCoeff = settings.get<double>("Model.PropWashCoeff");
    modelSettings.maxElevatorLift = settings.get<double>("Model.MaxElevatorLift");
    modelSettings.maxElevatorAngleRadians =
        settings.get<double>("Model.MaxElevatorAngleDegrees") * boost::math::double_constants::pi / 180.0;

    return modelSettings;
}

int main(int argc, char** argv)
{
    initLogging();

    SetConsoleCtrlHandler(CtrlHandler, TRUE);

    QueueRunner runner;  // setup runner for this thread (will be started later)

    // read settings
    spdlog::info("Starting settings reading and parsing");

    path exeName = argv[0];
    path exePath = exeName.parent_path();
    ptree settings = readSettings(exePath);

    bffcl::UDPClient::Settings clSettings = readCLSettings(settings);
    Sim::Settings simSettings = readSimSettings(settings);
    Model::Settings modelSettings = readModelSettings(settings);

    spdlog::info("Settings read and parsed successfully");

    // create main components
    spdlog::info("Creating main components");

    bffcl::UDPClient cl(clSettings);
    Sim sim(simSettings);
    Model model(modelSettings);

    spdlog::info("Main components created successfully");

    // update CL defaults from model
    auto& input = cl.lockInput();
    input.aileron.frictionCoeff = model.getFrictionCoeff(Model::Aileron);
    input.aileron.dumpingCoeff = model.getDumpingCoeff(Model::Aileron);
    input.elevator.frictionCoeff = model.getFrictionCoeff(Model::Elevator);
    input.elevator.dumpingCoeff = model.getDumpingCoeff(Model::Elevator);
    cl.unlockInput();

    Timer simModelLoop(std::chrono::milliseconds(1000 / 30), runner, [&cl, &sim, &model] {  // run at 30Hz

        // 0. updae sim
        sim.process();

        // 1. pass values to sim
        const bffcl::CLReturn& output = cl.lockOutput();
        float elevator = output.axisElevatorPosition;
        float aileron = output.axisAileronPosition;
        cl.unlockOutput();

        sim.writeElevator(elevator);
        sim.writeAileron(aileron);

        // 2. read values from CL and Sim => send to model
        model.setAileron(aileron);
        model.setElevator(elevator);

        model.setTAS(sim.readTAS());
        model.setThrust(sim.readThrust());
        model.setCLElevatorTrim(sim.readCLElevatorTrim());

        // 3. update model
        model.process();

        // 4. write model calculation results to CL
        auto& input = cl.lockInput();

        input.elevator.fixedForce = model.getFixedForce(Model::Elevator);
        input.elevator.springForce = model.getSpringForce(Model::Elevator);
        input.elevator.vibrationCh1Hz = model.getVibrationCh1Hz(Model::Elevator);
        input.elevator.vibrationCh1Amp = model.getVibrationCh1Amp(Model::Elevator);
        input.elevator.vibrationCh2Hz = model.getVibrationCh2Hz(Model::Elevator);
        input.elevator.vibrationCh2Amp = model.getVibrationCh2Amp(Model::Elevator);
        input.elevator.vibrationCh3Hz = model.getVibrationCh2Hz(Model::Elevator);
        input.elevator.vibrationCh3Amp = model.getVibrationCh3Amp(Model::Elevator);

        input.aileron.fixedForce = model.getFixedForce(Model::Aileron);
        input.aileron.springForce = model.getSpringForce(Model::Aileron);
        input.aileron.vibrationCh1Hz = model.getVibrationCh1Hz(Model::Aileron);
        input.aileron.vibrationCh1Amp = model.getVibrationCh1Amp(Model::Aileron);
        input.aileron.vibrationCh2Hz = model.getVibrationCh2Hz(Model::Aileron);
        input.aileron.vibrationCh2Amp = model.getVibrationCh2Amp(Model::Aileron);
        input.aileron.vibrationCh3Hz = model.getVibrationCh2Hz(Model::Aileron);
        input.aileron.vibrationCh3Amp = model.getVibrationCh3Amp(Model::Aileron);

        cl.unlockInput();

      

        return false;  // false means call again
    });

    spdlog::info("Starting sim/model loop");
    simModelLoop.start();
    spdlog::info("sim/model loop started");

    spdlog::info("Starting main loop");
    runner.run();
    spdlog::info("Main loop finished");

    return 0;
}
