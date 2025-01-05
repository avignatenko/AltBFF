#include "ReadSettings.h"

#include <map>

using std::filesystem::file_time_type;
using std::filesystem::path;

toml::value readSettings(const path& file)
{
    return toml::parse(file);
}

LogSettings readLogSettings(const toml::value& settings)
{
    LogSettings logSettings;

    auto logLevelStr = settings.at("App").at("LogLevel").as_string();

    std::map<std::string, spdlog::level::level_enum> settingsStrToEnum = {
        {"trace", spdlog::level::trace}, {"debug", spdlog::level::debug}, {"info", spdlog::level::info},
        {"warn", spdlog::level::warn},   {"err", spdlog::level::err},     {"critical", spdlog::level::critical},
        {"off", spdlog::level::off}};

    if (auto levelFound = settingsStrToEnum.find(logLevelStr); levelFound != settingsStrToEnum.end())
        logSettings.logLevel = levelFound->second;

    return logSettings;
}

bffcl::UDPClient::Settings readCLSettings(const toml::value& settings)
{
    bffcl::UDPClient::Settings clSettings;
    auto networkTable = settings.at("Network");
    clSettings.toAddress = networkTable.at("CLIPAddress").as_string();
    clSettings.toPort = networkTable.at("CLPort").as_integer();
    clSettings.fromAddress = networkTable.at("ThisIPAddress").as_string();
    clSettings.fromPort = networkTable.at("ThisPort").as_integer();
    clSettings.sendFreq = networkTable.at("SendFreqHz").as_floating();

    return clSettings;
}

SimFSUIPC::Settings readSimSettings(const toml::value& settings)
{
    SimFSUIPC::Settings simSettings;

    const auto simTable = settings.at("Sim");

    simSettings.invertFSElevator = simTable.at("InvertElevator").as_boolean();
    simSettings.invertFSAileron = simTable.at("InvertAileron").as_boolean();
    simSettings.invertCLElevatorTrim = simTable.at("InvertCLElevatorTrim").as_boolean();
    simSettings.propWashOffset = simTable.at("PropWashOffset").as_integer();
    simSettings.clElevatorTrimOffset = simTable.at("CLElevatorTrimOffset").as_integer();
    simSettings.apRollEngagedOffset = simTable.at("APRollEngagedOffset").as_integer();
    simSettings.apPitchEngagedOffset = simTable.at("APPitchEngagedOffset").as_integer();
    simSettings.apPitchLimitsOffset = simTable.at("APPitchLimitsOffset").as_integer();
    simSettings.clForceEnabledOffset = simTable.at("CLForceEnabledOffset").as_integer();
    simSettings.clEngageOffset = simTable.at("CLEngageOffset").as_integer();
    return simSettings;
}

A2AStec30AP::Settings readAPSettings(const toml::value& settings)
{
    A2AStec30AP::Settings apSettings;

    const auto apTable = settings.at("AP");

    apSettings.pitchmode = apTable.at("PitchMode").as_integer();

    apSettings.elevatorServoDuMax = apTable.at("ElevatorServoDuMax").as_integer();

    apSettings.elevatorPID.p = apTable.at("ElevatorP").as_floating();
    apSettings.elevatorPID.i = apTable.at("ElevatorI").as_floating();
    apSettings.elevatorPID.d = apTable.at("ElevatorD").as_floating();
    apSettings.elevatorDuMax = apTable.at("ElevatorDuMax").as_floating();

    apSettings.pitchPID.p = apTable.at("PitchP").as_floating();
    apSettings.pitchPID.i = apTable.at("PitchI").as_floating();
    apSettings.pitchPID.d = apTable.at("PitchD").as_floating();
    apSettings.pitchMax = degToRad(apTable.at("PitchMaxDeg").as_floating());
    apSettings.pitchDuMax = apTable.at("PitchDuMax").as_floating();

    apSettings.fpmPID.p = apTable.at("FpmP").as_floating();
    apSettings.fpmPID.i = apTable.at("FpmI").as_floating();
    apSettings.fpmPID.d = apTable.at("FpmD").as_floating();
    apSettings.fpmMax = apTable.at("FpmMax").as_floating();
    apSettings.fpmDuMax = apTable.at("FpmDuMax").as_floating();

    apSettings.pitchWarningCLForce = apTable.at("PitchWarningCLForce").as_floating();
    apSettings.pitchMaxCLForce = apTable.at("PitchMaxCLForce").as_floating();
    apSettings.pitchStartDegradeCLForce = apTable.at("PitchStartDegradeCLForce").as_floating();

    apSettings.doStepResponse = apTable.at("DoStepResponse").as_boolean();
    apSettings.stepResponseInputFile = apTable.at("StepResponseInputFile").as_string();
    return apSettings;
}

Model::Settings readModelSettings(const toml::value& settings)
{
    Model::Settings modelSettings;

    const auto modelTable = settings.at("Model");

    modelSettings.hardware.aileronFrictionCoeff = modelTable.at("AileronFrictionCoeff").as_integer();
    modelSettings.hardware.aileronDumpingCoeff = modelTable.at("AileronDumpingCoeff").as_integer();
    modelSettings.hardware.aileronPositionFollowingP = modelTable.at("AileronPositionFollowingP").as_integer();
    modelSettings.hardware.aileronPositionFollowingI = modelTable.at("AileronPositionFollowingI").as_integer();
    modelSettings.hardware.aileronPositionFollowingD = modelTable.at("AileronPositionFollowingD").as_integer();

    modelSettings.hardware.elevatorFrictionCoeff = modelTable.at("ElevatorFrictionCoeff").as_integer();
    modelSettings.hardware.elevatorDumpingCoeff = modelTable.at("ElevatorDumpingCoeff").as_integer();
    modelSettings.hardware.elevatorPositionFollowingP = modelTable.at("ElevatorPositionFollowingP").as_integer();
    modelSettings.hardware.elevatorPositionFollowingI = modelTable.at("ElevatorPositionFollowingI").as_integer();
    modelSettings.hardware.elevatorPositionFollowingD = modelTable.at("ElevatorPositionFollowingD").as_integer();

    modelSettings.clExponent = modelTable.at("CLExponent").as_floating();

    modelSettings.hTailPosLon = modelTable.at("HTailPosLon").as_floating();
    modelSettings.wingRootChord = modelTable.at("WingRootChord").as_floating();

    modelSettings.elevatorArea = modelTable.at("ElevatorArea").as_floating();
    modelSettings.elevatorNeutralPos = modelTable.at("ElevatorNeutralPos").as_floating();
    modelSettings.elevatorTrimGain = modelTable.at("ElevatorTrimGain").as_floating();
    modelSettings.elevatorTrimNeutralPos = modelTable.at("ElevatorTrimNeutralPos").as_floating();
    modelSettings.propWashElevatorCoeff = modelTable.at("PropWashElevatorCoeff").as_floating();
    modelSettings.calculatePropWash = modelTable.at("CalculatePropWash").as_boolean();

    modelSettings.elevatorAlphaGain = modelTable.at("ElevatorAlphaGain").as_floating();
    modelSettings.elevatorAlphaScaleSpeedKn = modelTable.at("ElevatorAlphaScaleSpeedKn").as_floating();

    modelSettings.elevatorPRGain = modelTable.at("ElevatorPRGain").as_floating();
    modelSettings.maxElevatorLift = modelTable.at("MaxElevatorLift").as_floating();
    modelSettings.maxElevatorAngleRadians = degToRad(modelTable.at("MaxElevatorAngleDegrees").as_floating());

    modelSettings.engineVibAirGain = modelTable.at("EngineVibAirGain").as_floating();

    modelSettings.elevatorEngineFlowGain = modelTable.at("ElevatorEngineFlowGain").as_floating();
    modelSettings.elevatorEngineFreqGain = modelTable.at("ElevatorEngineFreqGain").as_floating();
    modelSettings.elevatorEngineFreqMin = modelTable.at("ElevatorEngineFreqMin").as_floating();

    modelSettings.elevatorVibStallGain = modelTable.at("ElevatorVibStallGain").as_floating();
    modelSettings.elevatorVibStalFreq = modelTable.at("ElevatorVibStalFreq").as_floating();

    modelSettings.elevatorVibRunwayGain = modelTable.at("ElevatorVibRunwayGain").as_floating();
    modelSettings.elevatorVibRunwayFreq = modelTable.at("ElevatorVibRunwayFreq").as_floating();

    modelSettings.aileronArea = modelTable.at("AileronArea").as_floating();
    modelSettings.aileronTrimGain = modelTable.at("AileronTrimGain").as_floating();
    modelSettings.propWashAileronCoeff = modelTable.at("PropWashAileronCoeff").as_floating();
    modelSettings.maxAileronLift = modelTable.at("MaxAileronLift").as_floating();
    modelSettings.maxAileronAngleRadians = degToRad(modelTable.at("MaxAileronAngleDegrees").as_floating());

    modelSettings.aileronEngineFlowGain = modelTable.at("AileronEngineFlowGain").as_floating();
    modelSettings.aileronEngineFreqGain = modelTable.at("AileronEngineFreqGain").as_floating();
    modelSettings.aileronEngineFreqMin = modelTable.at("AileronEngineFreqMin").as_floating();

    modelSettings.aileronVibStallGain = modelTable.at("AileronVibStallGain").as_floating();
    modelSettings.aileronVibStalFreq = modelTable.at("AileronVibStalFreq").as_floating();

    modelSettings.aileronVibRunwayGain = modelTable.at("AileronVibRunwayGain").as_floating();
    modelSettings.aileronVibRunwayFreq = modelTable.at("AileronVibRunwayFreq").as_floating();

    modelSettings.forceTrimIntoSim = modelTable.at("ForceTrimIntoSim").as_boolean();
    modelSettings.forcedSimTrim = modelTable.at("ForcedSimTrim").as_floating();

    return modelSettings;
}
