
#include "A2ASTec30AP.h"
#include "Model.h"
#include "Sim.h"

#include <BFFCLAPI/UDPClient.h>

#include <Utils/Common.h>
#include <Utils/QueueRunner.h>
#include <Utils/Timer.h>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

#include <boost/algorithm/string.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#define CATCH_CONFIG_RUNNER
#include <catch2/catch.hpp>

#include <windows.h>

#include <stdio.h>
#include <filesystem>
#include <iostream>
#include <string>

using namespace std;

BOOL WINAPI CtrlHandler(DWORD fdwCtrlType)
{
    // todo: add closing
    spdlog::shutdown();

    return FALSE;  // let others work on this
}

using boost::property_tree::ptree;
using std::filesystem::file_time_type;
using std::filesystem::path;

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

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("altbff.log", true);
    file_sink->set_level(settings.logLevel);

    auto logger = std::make_shared<spdlog::logger>("multi_sink", spdlog::sinks_init_list{console_sink, file_sink});

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
    clSettings.sendFreq = settings.get<double>("Network.SendFreqHz");

    return clSettings;
}

Sim::Settings readSimSettings(const ptree& settings)
{
    Sim::Settings simSettings;
    simSettings.invertFSElevator = settings.get<bool>("Sim.InvertElevator");
    simSettings.invertFSAileron = settings.get<bool>("Sim.InvertAileron");
    simSettings.invertCLElevatorTrim = settings.get<bool>("Sim.InvertCLElevatorTrim");
    simSettings.propWashOffset = std::stoul(settings.get<std::string>("Sim.PropWashOffset"), nullptr, 16);
    simSettings.clElevatorTrimOffset = std::stoul(settings.get<std::string>("Sim.CLElevatorTrimOffset"), nullptr, 16);
    simSettings.apRollEngagedOffset = std::stoul(settings.get<std::string>("Sim.APRollEngagedOffset"), nullptr, 16);
    simSettings.apPitchEngagedOffset = std::stoul(settings.get<std::string>("Sim.APPitchEngagedOffset"), nullptr, 16);
    simSettings.apPitchLimitsOffset = std::stoul(settings.get<std::string>("Sim.APPitchLimitsOffset"), nullptr, 16);
    simSettings.clForceEnabledOffset = std::stoul(settings.get<std::string>("Sim.CLForceEnabledOffset"), nullptr, 16);
    simSettings.clEngageOffset = std::stoul(settings.get<std::string>("Sim.CLEngageOffset"), nullptr, 16);
    return simSettings;
}

A2AStec30AP::Settings readAPSettings(const ptree& settings)
{
    A2AStec30AP::Settings apSettings;

    apSettings.pitchmode = settings.get<int>("AP.PitchMode");

    apSettings.elevatorServoDuMax = settings.get<int>("AP.ElevatorServoDuMax");

    apSettings.elevatorPID.p = settings.get<double>("AP.ElevatorP");
    apSettings.elevatorPID.i = settings.get<double>("AP.ElevatorI");
    apSettings.elevatorPID.d = settings.get<double>("AP.ElevatorD");
    apSettings.elevatorDuMax = settings.get<double>("AP.ElevatorDuMax");

    apSettings.pitchPID.p = settings.get<double>("AP.PitchP");
    apSettings.pitchPID.i = settings.get<double>("AP.PitchI");
    apSettings.pitchPID.d = settings.get<double>("AP.PitchD");
    apSettings.pitchMax = degToRad(settings.get<double>("AP.PitchMaxDeg"));
    apSettings.pitchDuMax = settings.get<double>("AP.PitchDuMax");

    apSettings.fpmPID.p = settings.get<double>("AP.FpmP");
    apSettings.fpmPID.i = settings.get<double>("AP.FpmI");
    apSettings.fpmPID.d = settings.get<double>("AP.FpmD");
    apSettings.fpmMax = settings.get<double>("AP.FpmMax");
    apSettings.fpmDuMax = settings.get<double>("AP.FpmDuMax");

    apSettings.pitchWarningCLForce = settings.get<double>("AP.PitchWarningCLForce");
    apSettings.pitchMaxCLForce = settings.get<double>("AP.PitchMaxCLForce");
    apSettings.pitchStartDegradeCLForce = settings.get<double>("AP.PitchStartDegradeCLForce");

    apSettings.doStepResponse = settings.get<bool>("AP.DoStepResponse");
    apSettings.stepResponseInputFile = settings.get<std::string>("AP.StepResponseInputFile");
    return apSettings;
}

Model::Settings readModelSettings(const ptree& settings)
{
    Model::Settings modelSettings;
    modelSettings.hardware.aileronFrictionCoeff = settings.get<int>("Model.AileronFrictionCoeff");
    modelSettings.hardware.aileronDumpingCoeff = settings.get<int>("Model.AileronDumpingCoeff");
    modelSettings.hardware.aileronPositionFollowingP = settings.get<int>("Model.AileronPositionFollowingP");
    modelSettings.hardware.aileronPositionFollowingI = settings.get<int>("Model.AileronPositionFollowingI");
    modelSettings.hardware.aileronPositionFollowingD = settings.get<int>("Model.AileronPositionFollowingD");

    modelSettings.hardware.elevatorFrictionCoeff = settings.get<int>("Model.ElevatorFrictionCoeff");
    modelSettings.hardware.elevatorDumpingCoeff = settings.get<int>("Model.ElevatorDumpingCoeff");
    modelSettings.hardware.elevatorPositionFollowingP = settings.get<int>("Model.ElevatorPositionFollowingP");
    modelSettings.hardware.elevatorPositionFollowingI = settings.get<int>("Model.ElevatorPositionFollowingI");
    modelSettings.hardware.elevatorPositionFollowingD = settings.get<int>("Model.ElevatorPositionFollowingD");

    modelSettings.clExponent = settings.get<double>("Model.CLExponent");

    modelSettings.hTailPosLon = settings.get<double>("Model.HTailPosLon");
    modelSettings.wingRootChord = settings.get<double>("Model.WingRootChord");

    modelSettings.elevatorArea = settings.get<double>("Model.ElevatorArea");
    modelSettings.elevatorNeutralPos = settings.get<double>("Model.ElevatorNeutralPos");
    modelSettings.elevatorTrimGain = settings.get<double>("Model.ElevatorTrimGain");
    modelSettings.elevatorTrimNeutralPos = settings.get<double>("Model.ElevatorTrimNeutralPos");
    modelSettings.propWashElevatorCoeff = settings.get<double>("Model.PropWashElevatorCoeff");
    modelSettings.calculatePropWash = settings.get<bool>("Model.CalculatePropWash");
    
    modelSettings.elevatorAlphaGain = settings.get<double>("Model.ElevatorAlphaGain");
    modelSettings.elevatorAlphaScaleSpeedKn = settings.get<double>("Model.ElevatorAlphaScaleSpeedKn");

    modelSettings.elevatorPRGain = settings.get<double>("Model.ElevatorPRGain");
    modelSettings.maxElevatorLift = settings.get<double>("Model.MaxElevatorLift");
    modelSettings.maxElevatorAngleRadians = degToRad(settings.get<double>("Model.MaxElevatorAngleDegrees"));

    modelSettings.engineVibAirGain = settings.get<double>("Model.EngineVibAirGain");

    modelSettings.elevatorEngineFlowGain = settings.get<double>("Model.ElevatorEngineFlowGain");
    modelSettings.elevatorEngineFreqGain = settings.get<double>("Model.ElevatorEngineFreqGain");
    modelSettings.elevatorEngineFreqMin = settings.get<double>("Model.ElevatorEngineFreqMin");

    modelSettings.elevatorVibStallGain = settings.get<double>("Model.ElevatorVibStallGain");
    modelSettings.elevatorVibStalFreq = settings.get<double>("Model.ElevatorVibStalFreq");

    modelSettings.elevatorVibRunwayGain = settings.get<double>("Model.ElevatorVibRunwayGain");
    modelSettings.elevatorVibRunwayFreq = settings.get<double>("Model.ElevatorVibRunwayFreq");

    modelSettings.aileronArea = settings.get<double>("Model.AileronArea");
    modelSettings.aileronTrimGain = settings.get<double>("Model.AileronTrimGain");
    modelSettings.propWashAileronCoeff = settings.get<double>("Model.PropWashAileronCoeff");
    modelSettings.maxAileronLift = settings.get<double>("Model.MaxAileronLift");
    modelSettings.maxAileronAngleRadians = degToRad(settings.get<double>("Model.MaxAileronAngleDegrees"));

    modelSettings.aileronEngineFlowGain = settings.get<double>("Model.AileronEngineFlowGain");
    modelSettings.aileronEngineFreqGain = settings.get<double>("Model.AileronEngineFreqGain");
    modelSettings.aileronEngineFreqMin = settings.get<double>("Model.AileronEngineFreqMin");

    modelSettings.aileronVibStallGain = settings.get<double>("Model.AileronVibStallGain");
    modelSettings.aileronVibStalFreq = settings.get<double>("Model.AileronVibStalFreq");

    modelSettings.aileronVibRunwayGain = settings.get<double>("Model.AileronVibRunwayGain");
    modelSettings.aileronVibRunwayFreq = settings.get<double>("Model.AileronVibRunwayFreq");

    return modelSettings;
}

void updateCLDefaultsFromModel(bffcl::UDPClient& cl, Model& model)
{
    auto& input = cl.lockInput();
    input.aileron.frictionCoeff = model.getFrictionCoeff(Model::Aileron);
    input.aileron.dumpingCoeff = model.getDumpingCoeff(Model::Aileron);

    input.aileron.positionFollowingP = model.getPositionFollowingP(Model::Aileron);
    input.aileron.positionFollowingI = model.getPositionFollowingI(Model::Aileron);
    input.aileron.positionFollowingD = model.getPositionFollowingD(Model::Aileron);

    input.elevator.frictionCoeff = model.getFrictionCoeff(Model::Elevator);
    input.elevator.dumpingCoeff = model.getDumpingCoeff(Model::Elevator);

    input.elevator.positionFollowingP = model.getPositionFollowingP(Model::Elevator);
    input.elevator.positionFollowingI = model.getPositionFollowingI(Model::Elevator);
    input.elevator.positionFollowingD = model.getPositionFollowingD(Model::Elevator);

    cl.unlockInput();
}

int runTests(int argc, char** argv)
{
    // remove "test" from command line
    int argc2 = argc - 1;
    std::vector<char*> argv2;
    for (int i = 0; i < argc; ++i)
        if (i != 1) argv2.push_back(argv[i]);

    int result = Catch::Session().run(argc2, &argv2[0]);
    return result;
}

int checkedMain(int argc, char** argv)
{
    path exeName = argv[0];
    path settingsPath = exeName.parent_path() / "settings.ini";

    ptree settings = readSettings(settingsPath);

    bffcl::UDPClient::Settings clSettings = readCLSettings(settings);
    Sim::Settings simSettings = readSimSettings(settings);
    Model::Settings modelSettings = readModelSettings(settings);
    LogSettings logSettings = readLogSettings(settings);
    A2AStec30AP::Settings apSettings = readAPSettings(settings);

    initLogging(logSettings);

    // create main components
    spdlog::info("Creating main components");

    bffcl::UDPClient cl(clSettings);
    Sim sim(simSettings);
    Model model(modelSettings);
    A2AStec30AP autopilot(apSettings);

    spdlog::info("Main components created successfully");

    updateCLDefaultsFromModel(cl, model);

    QueueRunner runner;  // setup runner for this thread

    auto kModelLoopFreq = std::chrono::milliseconds(1000 / 30);  // run at 30Hz
    Timer simModelLoop(kModelLoopFreq, runner, [&cl, &sim, &model, &autopilot] {
        // 0. deal with pause first of all
        if (sim.simulationPaused())
        {
            // send CL disengage
            auto& input = cl.lockInput();
            input.loadingEngage = 0;
            cl.unlockInput();

            // we only process sim to update pause status
            sim.process();

            // and that's it until simulation not restored
            return false;  // false means call again
        }

        // 1. read CL data
        const bffcl::CLReturn& output = cl.lockOutput();
        float elevatorCL = output.axisElevatorPosition;
        float aileronCL = output.axisAileronPosition;
        bool clForceEnabled = output.forceEnableStatus;
        cl.unlockOutput();

        // 2. read values from CL and Sim => send to model
        model.setAileron(sim.readAxisControlState(Sim::Aileron) == Sim::AxisControl::Manual
                             ? aileronCL
                             : autopilot.getCLAileron().value_or(aileronCL));

        model.setElevator(sim.readAxisControlState(Sim::Elevator) == Sim::AxisControl::Manual
                              ? elevatorCL
                              : autopilot.getCLElevator().value_or(elevatorCL));

        model.setAirDensity(sim.readAmbientAirDensity());
        model.setTAS(sim.readTAS());
        model.setGS(sim.readGS());
        model.setOnGround(sim.readOnGround());
        model.setGroundType(static_cast<Model::GroundType>(sim.readGroundType()));
        model.setThrust(sim.readThrust());
        model.setAlpha(sim.readAlpha());
        model.setElevatorTrim(sim.readCLElevatorTrim());
        model.setPitchRate(sim.readPitchRate());
        model.setCGPosFrac(sim.readCGPosFrac());
        model.setEngine1RPM(sim.readEngine1RPM());
        model.setEngine1Flow(sim.readEngine1Flow());
        model.setRelativeAoA(sim.readRelativeAoA());
        model.setPropWash(sim.readPropWash());

        // 3. update model
        model.process();

        // 3.1 update autopilot
        autopilot.enablePitchAxis(sim.readAxisControlState(Sim::Elevator) != Sim::AxisControl::Manual);
        autopilot.enableRollAxis(sim.readAxisControlState(Sim::Aileron) != Sim::AxisControl::Manual);
        autopilot.setSimAileron(sim.readAileron());
        autopilot.setSimElevator(elevatorCL);  // workaround!! wrong elevator value in sim :(
        autopilot.setPressureAltitude(sim.readPressureAltitude());
        autopilot.setSimPitch(sim.readPitch());
        autopilot.setSimFpm(sim.readFpm());
        autopilot.setTotalAxisCLForceAileron(model.getTotalForce(Model::Aileron));
        autopilot.setTotalAxisCLForceElevator(model.getTotalForce(Model::Elevator));

        autopilot.process();

        // get autopilot messages, etc.
        if (sim.readAxisControlState(Sim::Elevator) == Sim::AxisControl::Auto)
        {
            auto apWarning = autopilot.getTrimNeededWarning();

            Sim::APPitchLimits simPitchLimits = Sim::APPitchLimits::TrimOk;
            if (apWarning.warningLevel > 0)
            {
                if (apWarning.pitchDirection == A2AStec30AP::TrimNeededWarning::Down)
                    simPitchLimits = Sim::APPitchLimits::TrimDown;
                else if (apWarning.pitchDirection == A2AStec30AP::TrimNeededWarning::Up)
                    simPitchLimits = Sim::APPitchLimits::TrimUp;

                simPitchLimits = Sim::APPitchLimits(int(simPitchLimits) + apWarning.warningLevel - 1);

                spdlog::info("AP warning: Pitch {}! dforce: {}",
                             apWarning.pitchDirection == A2AStec30AP::TrimNeededWarning::Down ? "down" : "up",
                             apWarning.forceDelta);
            }

            sim.writeAPPitchLimits(simPitchLimits);
        }

        // 4. write model calculation results to CL
        auto& input = cl.lockInput();

        Sim::CLEngage simCLEngageCmd = sim.readCLEngage();
        if (simCLEngageCmd != Sim::CLEngage::NoChange)
            input.loadingEngage = (simCLEngageCmd == Sim::CLEngage::Engage ? 1 : 0);

        auto fixBFFPos = [](double pos) {
            // fix for BFF CL acception [0, 100] instead of [-100, 100]
            const float bffMin = 0;
            const float bffMax = 100;
            float bffFixPos = std::clamp(((float)pos + 100.0f) * (bffMax - bffMin) / 200.0f + bffMin, bffMin, bffMax);
            return bffFixPos;
        };

        if (sim.readAxisControlState(Sim::Elevator) == Sim::AxisControl::Manual)
        {
            input.positionFollowingEngage &= ~(1u << 0);  // clear pos following
        }
        else
        {
            input.positionFollowingEngage |= (1u << 0);  // set pos following
            input.elevator.positionFollowingSetPoint = fixBFFPos(autopilot.getCLElevator().value());
            spdlog::trace("Elevator in follow mode: {}", input.elevator.positionFollowingSetPoint);
        }

        input.elevator.fixedForce = model.getFixedForce(Model::Elevator);
        input.elevator.springForce = model.getSpringForce(Model::Elevator);

        input.elevator.vibrationCh1Hz = model.getVibrationEngineHz(Model::Elevator);
        input.elevator.vibrationCh1Amp = model.getVibrationEngineAmp(Model::Elevator);
        input.elevator.vibrationCh2Hz = model.getVibrationRunwayHz(Model::Elevator);
        input.elevator.vibrationCh2Amp = model.getVibrationRunwayAmp(Model::Elevator);
        input.elevator.vibrationCh3Hz = model.getVibrationStallHz(Model::Elevator);
        input.elevator.vibrationCh3Amp = model.getVibrationStallAmp(Model::Elevator);

        if (sim.readAxisControlState(Sim::Aileron) == Sim::AxisControl::Manual)
        {
            input.positionFollowingEngage &= ~(1u << 1);  // clear pos following
        }
        else
        {
            input.positionFollowingEngage |= (1u << 1);  // set pos following
            input.aileron.positionFollowingSetPoint = fixBFFPos(autopilot.getCLAileron().value());
            spdlog::trace("Aileron in follow mode: {}", input.aileron.positionFollowingSetPoint);
        }

        input.aileron.fixedForce = model.getFixedForce(Model::Aileron);
        input.aileron.springForce = model.getSpringForce(Model::Aileron);

        input.aileron.vibrationCh1Hz = model.getVibrationEngineHz(Model::Aileron);
        input.aileron.vibrationCh1Amp = model.getVibrationEngineAmp(Model::Aileron);
        input.aileron.vibrationCh2Hz = model.getVibrationRunwayHz(Model::Aileron);
        input.aileron.vibrationCh2Amp = model.getVibrationRunwayAmp(Model::Aileron);
        input.aileron.vibrationCh3Hz = model.getVibrationStallHz(Model::Aileron);
        input.aileron.vibrationCh3Amp = model.getVibrationStallAmp(Model::Aileron);

        cl.unlockInput();

        // 7. pass values to sim
        if (sim.readAxisControlState(Sim::Elevator) == Sim::AxisControl::Manual)
            sim.writeElevator(elevatorCL);
        else if (auto axisValue = autopilot.getSimElevator())
            sim.writeElevator(axisValue.value());

        if (sim.readAxisControlState(Sim::Aileron) == Sim::AxisControl::Manual)
            sim.writeAileron(aileronCL);
        else if (auto axisValue = autopilot.getSimAileron())
            sim.writeAileron(axisValue.value());

        sim.writeElevatorTrim(0.0);

        sim.writeCLForceEnabled(clForceEnabled);

        sim.process();

        return false;  // false means call again
    });

    // settings refresh utility
    file_time_type settingsWriteTime = last_write_time(settingsPath);
    auto kSettingsLoopFreq = std::chrono::milliseconds(2000);  // run at 0.5Hz
    Timer settingsUpdateLoop(kSettingsLoopFreq, runner, [&cl, &model, &autopilot, settingsPath, &settingsWriteTime] {
        file_time_type newSettingsWriteTime = last_write_time(settingsPath);
        if (newSettingsWriteTime > settingsWriteTime)
        {
            spdlog::info("Settings updated, reloading model settings...");
            ptree settings = readSettings(settingsPath);
            Model::Settings modelSettings = readModelSettings(settings);
            model.setSettings(modelSettings);

            updateCLDefaultsFromModel(cl, model);

            spdlog::info("Model settings updated");

            A2AStec30AP::Settings apSettings = readAPSettings(settings);
            autopilot.setSettings(apSettings);
            spdlog::info("AP settings updated");

            settingsWriteTime = newSettingsWriteTime;
        }
        return false;  // call again
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

int main(int argc, char** argv)
{
    // check for tests session
    if (argc > 1 && strcmp(argv[1], "test") == 0) return runTests(argc, argv);

    SetConsoleCtrlHandler(CtrlHandler, TRUE);

    try
    {
        return checkedMain(argc, argv);
    }
    catch (const std::exception& e)
    {
        spdlog::critical("Exception: {}", e.what());
        spdlog::shutdown();
        throw;
    }
    catch (...)
    {
        spdlog::critical("Unknown exception");
        spdlog::shutdown();
        throw;
    }
}
