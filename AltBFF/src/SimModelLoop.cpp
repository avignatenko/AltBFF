#include "SimModelLoop.h"

#include "A2ASTec30AP.h"
#include "Model.h"
#include "Sim.h"

#include <BFFCLAPI/UDPClient.h>

bool simModelLoop(bffcl::UDPClient& cl, Sim& sim, Model& model, A2AStec30AP& autopilot)
{
    // 0. deal with pause first of all
    if (sim.simulationPaused())
    {
        // send CL disengage
        auto& input = cl.lockInput();
        input.loadingEngage = 0;

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

    auto fixBFFPos = [](double pos)
    {
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

    // 7. pass values to sim
    if (sim.readAxisControlState(Sim::Elevator) == Sim::AxisControl::Manual)
        sim.writeElevator(elevatorCL);
    else if (auto axisValue = autopilot.getSimElevator())
        sim.writeElevator(axisValue.value());

    if (sim.readAxisControlState(Sim::Aileron) == Sim::AxisControl::Manual)
        sim.writeAileron(aileronCL);
    else if (auto axisValue = autopilot.getSimAileron())
        sim.writeAileron(axisValue.value());

    if (model.getForceTrimIntoSim()) sim.writeElevatorTrim(model.getForcedSimElevatorTrim());

    sim.writeCLForceEnabled(clForceEnabled);

    sim.process();

    return false;  // false means call again
}
