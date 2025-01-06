#include "ControlLoop.h"

#include "A2ASTec30AP.h"
#include "Model.h"
#include "Sim.h"

#include <BFFCLAPI/UDPClient.h>

void ControlLoop::process()
{
    // 0. deal with pause first of all
    if (sim_.simulationPaused())
    {
        // send CL disengage
        auto& input = cl_.lockInput();
        input.loadingEngage = 0;

        // and that's it until simulation not restored
        return;
    }

    // 1. read CL data
    const bffcl::CLReturn& output = cl_.lockOutput();
    float elevatorCL = output.axisElevatorPosition;
    float aileronCL = output.axisAileronPosition;
    bool clForceEnabled = output.forceEnableStatus;

    // 2. read values from CL and Sim => send to model
    model_.setAileron(sim_.readAxisControlState(Sim::Aileron) == Sim::AxisControl::Manual
                          ? aileronCL
                          : autopilot_.getCLAileron().value_or(aileronCL));

    model_.setElevator(sim_.readAxisControlState(Sim::Elevator) == Sim::AxisControl::Manual
                           ? elevatorCL
                           : autopilot_.getCLElevator().value_or(elevatorCL));

    model_.setAirDensity(sim_.readAmbientAirDensity());
    model_.setTAS(sim_.readTAS());
    model_.setGS(sim_.readGS());
    model_.setOnGround(sim_.readOnGround());
    model_.setGroundType(static_cast<Model::GroundType>(sim_.readGroundType()));
    model_.setThrust(sim_.readThrust());
    model_.setAlpha(sim_.readAlpha());
    model_.setElevatorTrim(sim_.readCLElevatorTrim());
    model_.setPitchRate(sim_.readPitchRate());
    model_.setCGPosFrac(sim_.readCGPosFrac());
    model_.setEngine1RPM(sim_.readEngine1RPM());
    model_.setEngine1Flow(sim_.readEngine1Flow());
    model_.setRelativeAoA(sim_.readRelativeAoA());
    model_.setPropWash(sim_.readPropWash());

    // 3.1 update autopilot
    autopilot_.enablePitchAxis(sim_.readAxisControlState(Sim::Elevator) != Sim::AxisControl::Manual);
    autopilot_.enableRollAxis(sim_.readAxisControlState(Sim::Aileron) != Sim::AxisControl::Manual);
    autopilot_.setSimAileron(sim_.readAileron());
    autopilot_.setSimElevator(elevatorCL);  // workaround!! wrong elevator value in sim :(
    autopilot_.setPressureAltitude(sim_.readPressureAltitude());
    autopilot_.setSimPitch(sim_.readPitch());
    autopilot_.setSimFpm(sim_.readFpm());
    autopilot_.setTotalAxisCLForceAileron(model_.getTotalForce(Model::Aileron));
    autopilot_.setTotalAxisCLForceElevator(model_.getTotalForce(Model::Elevator));

    // get autopilot messages, etc.
    if (sim_.readAxisControlState(Sim::Elevator) == Sim::AxisControl::Auto)
    {
        auto apWarning = autopilot_.getTrimNeededWarning();

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

        sim_.writeAPPitchLimits(simPitchLimits);
    }

    // 4. write model calculation results to CL
    auto& input = cl_.lockInput();

    Sim::CLEngage simCLEngageCmd = sim_.readCLEngage();
    if (simCLEngageCmd != Sim::CLEngage::NoChange)
        input.loadingEngage = (simCLEngageCmd == Sim::CLEngage::Engage ? 1 : 0);

    auto fixBFFPos = [](double pos)
    {
        // fix for BFF CL acception [0, 100] instead of [-100, 100]
        const float bffMin = 0;
        const float bffMax = 100;
        float bffFixPos = std::clamp(((float)pos + 100.0f) * (bffMax - bffMin) / 200.0f + bffMin, bffMin, bffMax);
        return bffFixPos;
    };

    if (sim_.readAxisControlState(Sim::Elevator) == Sim::AxisControl::Manual)
    {
        input.positionFollowingEngage &= ~(1u << 0);  // clear pos following
    }
    else
    {
        input.positionFollowingEngage |= (1u << 0);  // set pos following
        input.elevator.positionFollowingSetPoint = fixBFFPos(autopilot_.getCLElevator().value());
        spdlog::trace("Elevator in follow mode: {}", input.elevator.positionFollowingSetPoint);
    }

    input.elevator.fixedForce = model_.getFixedForce(Model::Elevator);
    input.elevator.springForce = model_.getSpringForce(Model::Elevator);

    input.elevator.vibrationCh1Hz = model_.getVibrationEngineHz(Model::Elevator);
    input.elevator.vibrationCh1Amp = model_.getVibrationEngineAmp(Model::Elevator);
    input.elevator.vibrationCh2Hz = model_.getVibrationRunwayHz(Model::Elevator);
    input.elevator.vibrationCh2Amp = model_.getVibrationRunwayAmp(Model::Elevator);
    input.elevator.vibrationCh3Hz = model_.getVibrationStallHz(Model::Elevator);
    input.elevator.vibrationCh3Amp = model_.getVibrationStallAmp(Model::Elevator);

    if (sim_.readAxisControlState(Sim::Aileron) == Sim::AxisControl::Manual)
    {
        input.positionFollowingEngage &= ~(1u << 1);  // clear pos following
    }
    else
    {
        input.positionFollowingEngage |= (1u << 1);  // set pos following
        input.aileron.positionFollowingSetPoint = fixBFFPos(autopilot_.getCLAileron().value());
        spdlog::trace("Aileron in follow mode: {}", input.aileron.positionFollowingSetPoint);
    }

    input.aileron.fixedForce = model_.getFixedForce(Model::Aileron);
    input.aileron.springForce = model_.getSpringForce(Model::Aileron);

    input.aileron.vibrationCh1Hz = model_.getVibrationEngineHz(Model::Aileron);
    input.aileron.vibrationCh1Amp = model_.getVibrationEngineAmp(Model::Aileron);
    input.aileron.vibrationCh2Hz = model_.getVibrationRunwayHz(Model::Aileron);
    input.aileron.vibrationCh2Amp = model_.getVibrationRunwayAmp(Model::Aileron);
    input.aileron.vibrationCh3Hz = model_.getVibrationStallHz(Model::Aileron);
    input.aileron.vibrationCh3Amp = model_.getVibrationStallAmp(Model::Aileron);

    // 7. pass values to sim
    if (sim_.readAxisControlState(Sim::Elevator) == Sim::AxisControl::Manual)
        sim_.writeElevator(elevatorCL);
    else if (auto axisValue = autopilot_.getSimElevator())
        sim_.writeElevator(axisValue.value());

    if (sim_.readAxisControlState(Sim::Aileron) == Sim::AxisControl::Manual)
        sim_.writeAileron(aileronCL);
    else if (auto axisValue = autopilot_.getSimAileron())
        sim_.writeAileron(axisValue.value());

    if (model_.getForceTrimIntoSim()) sim_.writeElevatorTrim(model_.getForcedSimElevatorTrim());

    sim_.writeCLForceEnabled(clForceEnabled);
}

void ControlLoop::updateCLDefaultsFromModel()
{
    auto& input = cl_.lockInput();
    input.aileron.frictionCoeff = model_.getFrictionCoeff(Model::Aileron);
    input.aileron.dumpingCoeff = model_.getDumpingCoeff(Model::Aileron);

    input.aileron.positionFollowingP = model_.getPositionFollowingP(Model::Aileron);
    input.aileron.positionFollowingI = model_.getPositionFollowingI(Model::Aileron);
    input.aileron.positionFollowingD = model_.getPositionFollowingD(Model::Aileron);

    input.elevator.frictionCoeff = model_.getFrictionCoeff(Model::Elevator);
    input.elevator.dumpingCoeff = model_.getDumpingCoeff(Model::Elevator);

    input.elevator.positionFollowingP = model_.getPositionFollowingP(Model::Elevator);
    input.elevator.positionFollowingI = model_.getPositionFollowingI(Model::Elevator);
    input.elevator.positionFollowingD = model_.getPositionFollowingD(Model::Elevator);
}
