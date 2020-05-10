
#include "Model.h"
#include "Sim.h"

#include <BFFCLAPI/UDPClient.h>

#include <Utils/QueueRunner.h>
#include <Utils/Timer.h>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/math/constants/constants.hpp>

#include <windows.h>

#include <filesystem>
#include <stdio.h>
#include <string>
#include <iostream>

using namespace std;

BOOL WINAPI CtrlHandler(DWORD fdwCtrlType)
{
    // todo: add closing

    return FALSE;  // let others work on this
}

using boost::property_tree::ptree;
using std::filesystem::path;
using std::filesystem::file_time_type;

struct LogSettings
{
    spdlog::level::level_enum logLevel = spdlog::level::info;
};

LogSettings readLogSettings(const ptree& settings)
{
    LogSettings logSettings;

    auto logLevelStr = boost::algorithm::to_lower_copy(settings.get<std::string>("App.LogLevel"));

    std::map<std::string, spdlog::level::level_enum> settingsStrToEnum = {
        {"trace", spdlog::level::trace}, {"debug", spdlog::level::debug}, {"info", spdlog::level::info},
        {"warn", spdlog::level::warn},   {"err", spdlog::level::err},     {"critical", spdlog::level::critical},
        {"off", spdlog::level::off}};

    if (auto levelFound = settingsStrToEnum.find(logLevelStr); levelFound != settingsStrToEnum.end())
        logSettings.logLevel = levelFound->second;

    return logSettings;
}

void initLogging(const LogSettings& settings)
{
    // init logging

    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);
 
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("altfs.log", true);
    file_sink->set_level(settings.logLevel);

    auto logger = std::make_shared<spdlog::logger>(
        "multi_sink", 
        spdlog::sinks_init_list{console_sink, file_sink});

    logger->set_level(spdlog::level::trace);
  
    spdlog::set_default_logger(logger);

    spdlog::info("AltBFF logging started");
}

ptree readSettings(const path& file)
{
    ptree settings;
    boost::property_tree::read_ini(file.string(), settings);
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
    simSettings.clElevatorTrimOffset = std::stoul(settings.get<std::string>("Sim.CLElevatorTrimOffset"), nullptr, 16);
    return simSettings;
}

Model::Settings readModelSettings(const ptree& settings)
{
    Model::Settings modelSettings;
    modelSettings.aileronFrictionCoeff = settings.get<int>("Model.AileronFrictionCoeff");
    modelSettings.aileronDumpingCoeff = settings.get<int>("Model.AileronDumpingCoeff");
    modelSettings.elevatorFrictionCoeff = settings.get<int>("Model.ElevatorFrictionCoeff");
    modelSettings.elevatorDumpingCoeff = settings.get<int>("Model.ElevatorDumpingCoeff");

    modelSettings.clExponent = settings.get<double>("Model.CLExponent");
  
    modelSettings.hTailPosLon = settings.get<double>("Model.HTailPosLon");
    modelSettings.wingRootChord = settings.get<double>("Model.WingRootChord");

    modelSettings.elevatorArea = settings.get<double>("Model.ElevatorArea");
    modelSettings.elevatorTrimGain = settings.get<double>("Model.ElevatorTrimGain");   
    modelSettings.propWashElevatorCoeff = settings.get<double>("Model.PropWashElevatorCoeff");
    modelSettings.elevatorAlphaGain = settings.get<double>("Model.ElevatorAlphaGain");
    modelSettings.elevatorPRGain = settings.get<double>("Model.ElevatorPRGain");
    modelSettings.maxElevatorLift = settings.get<double>("Model.MaxElevatorLift");
    modelSettings.maxElevatorAngleRadians =
        settings.get<double>("Model.MaxElevatorAngleDegrees") * boost::math::double_constants::pi / 180.0;

    modelSettings.elevatorEngineFlowGain = settings.get<double>("Model.ElevatorEngineFlowGain");
    modelSettings.elevatorEngineFreqGain = settings.get<double>("Model.ElevatorEngineFreqGain");
    modelSettings.elevatorEngineFreqMin = settings.get<double>("Model.ElevatorEngineFreqMin");

    modelSettings.aileronArea = settings.get<double>("Model.AileronArea");
    modelSettings.aileronTrimGain = settings.get<double>("Model.AileronTrimGain");
    modelSettings.propWashAileronCoeff = settings.get<double>("Model.PropWashAileronCoeff");
    modelSettings.maxAileronLift = settings.get<double>("Model.MaxAileronLift");
    modelSettings.maxAileronAngleRadians =
        settings.get<double>("Model.MaxAileronAngleDegrees") * boost::math::double_constants::pi / 180.0;

    modelSettings.aileronEngineFlowGain = settings.get<double>("Model.AileronEngineFlowGain");
    modelSettings.aileronEngineFreqGain = settings.get<double>("Model.AileronEngineFreqGain");
    modelSettings.aileronEngineFreqMin = settings.get<double>("Model.AileronEngineFreqMin");


    return modelSettings;
}

int main(int argc, char** argv)
{
    SetConsoleCtrlHandler(CtrlHandler, TRUE);

    QueueRunner runner;  // setup runner for this thread (will be started later)

    path exeName = argv[0];
    path settingsPath = exeName.parent_path() / "settings.ini";
   
    ptree settings = readSettings(settingsPath);

    bffcl::UDPClient::Settings clSettings = readCLSettings(settings);
    Sim::Settings simSettings = readSimSettings(settings);
    Model::Settings modelSettings = readModelSettings(settings);
    LogSettings logSettings = readLogSettings(settings);

    initLogging(logSettings);

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

    auto kModelLoopFreq = std::chrono::milliseconds(1000 / 30);  // run at 30Hz
    Timer simModelLoop(kModelLoopFreq, runner, [&cl, &sim, &model] { 

        // 1. read CL data
        const bffcl::CLReturn& output = cl.lockOutput();
        float elevator = output.axisElevatorPosition;
        float aileron = output.axisAileronPosition;
        cl.unlockOutput();

        // 2. pass values to sim 
        sim.writeElevator(elevator);
        sim.writeAileron(aileron);
        sim.writeElevatorTrim(0.0); // set default sim trim to zero (fixme: do once)

        // 2. read values from CL and Sim => send to model
        model.setAileron(aileron);
        model.setElevator(elevator);

        model.setAirDensity(sim.readAmbientAirDensity());
        model.setTAS(sim.readTAS());
        model.setGS(sim.readGS());
        model.setOnGround(sim.readOnGround());
        model.setThrust(sim.readThrust());
        model.setAlpha(sim.readAlpha());
        model.setElevatorTrim(sim.readCLElevatorTrim());
        model.setPitchRate(sim.readPitchRate());
        model.setCGPosFrac(sim.readCGPosFrac());
        model.setEngine1RPM(sim.readEngine1RPM());
        model.setEngine1Flow(sim.readEngine1Flow());

        // 3. update model and sim
        model.process();
        sim.process();
  
        // 4. write model calculation results to CL
        auto& input = cl.lockInput();

        input.elevator.fixedForce = model.getFixedForce(Model::Elevator);
        input.elevator.springForce = model.getSpringForce(Model::Elevator);
      
        input.elevator.vibrationCh1Hz = model.getVibrationCh1Hz(Model::Elevator);
        input.elevator.vibrationCh1Amp = model.getVibrationCh1Amp(Model::Elevator);
        input.elevator.vibrationCh2Hz = model.getVibrationCh2Hz(Model::Elevator);
        input.elevator.vibrationCh2Amp = model.getVibrationCh2Amp(Model::Elevator);
        input.elevator.vibrationCh3Hz = model.getVibrationCh3Hz(Model::Elevator);
        input.elevator.vibrationCh3Amp = model.getVibrationCh3Amp(Model::Elevator);

        input.aileron.fixedForce = model.getFixedForce(Model::Aileron);
        input.aileron.springForce = model.getSpringForce(Model::Aileron);
        
        input.aileron.vibrationCh1Hz = model.getVibrationCh1Hz(Model::Aileron);
        input.aileron.vibrationCh1Amp = model.getVibrationCh1Amp(Model::Aileron);
        input.aileron.vibrationCh2Hz = model.getVibrationCh2Hz(Model::Aileron);
        input.aileron.vibrationCh2Amp = model.getVibrationCh2Amp(Model::Aileron);
        input.aileron.vibrationCh3Hz = model.getVibrationCh3Hz(Model::Aileron);
        input.aileron.vibrationCh3Amp = model.getVibrationCh3Amp(Model::Aileron);

        cl.unlockInput();

        return false;  // false means call again
    });

    // settings refresh utility
    file_time_type settingsWriteTime = last_write_time(settingsPath);
    auto kSettingsLoopFreq = std::chrono::milliseconds(2000);  // run at 0.5Hz
    Timer settingsUpdateLoop(kSettingsLoopFreq, runner, [&model, settingsPath, &settingsWriteTime]
    {
        file_time_type newSettingsWriteTime = last_write_time(settingsPath);
        if (newSettingsWriteTime > settingsWriteTime)
        {
            spdlog::info("Settings updated, reloading model settings...");
            ptree settings = readSettings(settingsPath);
            Model::Settings modelSettings = readModelSettings(settings);
            model.setSettings(modelSettings);
            spdlog::info("Model settings updated");

            settingsWriteTime = newSettingsWriteTime;
        }
        return false; // call again
    });

    spdlog::info("Starting sim/model loop");
    simModelLoop.start();
    spdlog::info("sim/model loop started");

    spdlog::info("Starting settings refresh loop");
    settingsUpdateLoop.start();
    spdlog::info("settings refresh loop started");

    spdlog::info("Starting main loop");
    runner.run();
    spdlog::info("Main loop finished");

    return 0;
}
