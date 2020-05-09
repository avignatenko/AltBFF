#pragma once

#include <spdlog/spdlog.h>

#include <cstdint>

class Model
{
public:
    struct Settings
    {
        int aileronFrictionCoeff = 0;
        int aileronDumpingCoeff = 0;
        int elevatorFrictionCoeff = 0;
        int elevatorDumpingCoeff = 0;

        double clExponent = 0.0;

        double hTailPosLon = 0.0;
        double wingRootChord = 0.0;

        double elevatorArea = 0.0;
        double propWashElevatorCoeff = 0.0;
        double elevatorAlphaGain = 0.0;
        double elevatorPRGain = 0.0;
        double maxElevatorLift = 0.0;
        double maxElevatorAngleRadians = 0.0;
        double elevatorTrimGain = 0.0;

        double aileronArea = 0.0;
        double propWashAileronCoeff = 0.0;
        double maxAileronLift = 0.0;
        double maxAileronAngleRadians = 0.0;
        double aileronTrimGain = 0.0;

    };

    Model(const Settings& settings);

    void setSettings(const Settings& settings);

    enum Axis
    {
        Elevator = 0,
        Aileron,
        AxisCount,
    };

    // inputs

    // +/- 100%, mid = 0
    void setElevator(double elevator) 
    { 
        elevator_ = elevator; 
        spdlog::trace("Elevator set to model: {}", elevator_);
    }

    // +/- 100%, mid = 0
    void setAileron(double aileron) 
    { 
        aileron_ = aileron; 
        spdlog::trace("Aileron set to model: {}", aileron_);
    }

    // kg / m^3
    void setAirDensity(double density)
    {
        airDensity_ = density;
        spdlog::trace("Air density set to model: {}", airDensity_);
    }

    // tas in ms/sec
    void setTAS(double tas) 
    { 
        tas_ = tas; 
        spdlog::trace("TAS set to model: {}", tas_);
    }

    void setGS(double gs)
    {
        gs_ = gs;
        spdlog::trace("GS set to model: {}", gs_);
    }

    // true if on ground
    void setOnGround(bool onGround)
    {
        onGround_ = onGround;
        spdlog::trace("onGround set to model: {}", onGround_);
    }

    // thrust in pounds
    void setThrust(double thrust)
    { 
        thrust_ = thrust; 
        spdlog::trace("Thrust set to model: {}", thrust_);
    }


    // incidence "alpha" in radians
    void setAlpha(double alpha)
    {
        alphaAngleRad_ = alpha;
        spdlog::trace("Alpha set to model: {}", alphaAngleRad_);
    }

    // Pitch velocity in rads/sec relative to the body axes
    void setPitchRate(double pitchRate)
    {
        pitchRate_ = pitchRate;
        spdlog::trace("Pitch rate set to model: {}", pitchRate_);
    }

    void setCGPosFrac(double ccPosFrac)
    {
        cgPosFrac_ = ccPosFrac;
        spdlog::trace("GC Pos Frac rate set to model: {}", cgPosFrac_);
    }

    // [-1, 1]
    void setElevatorTrim(double clElevatorTrim) { 
        elevatorTrim_ = clElevatorTrim; 
        spdlog::trace("Elevator trim set to model: {}", elevatorTrim_);
    }
    
    // result

    int getFrictionCoeff(Axis axis);
    int getDumpingCoeff(Axis axis);

    float getFixedForce(Axis axis) { return fixedForce_[axis]; }
    float getSpringForce(Axis axis) { return springForce_[axis]; }
    uint16_t getVibrationCh1Hz(Axis axis) { return 0; }
    uint16_t getVibrationCh1Amp(Axis axis) { return 0; }
    uint16_t getVibrationCh2Hz(Axis axis) { return 0; }
    uint16_t getVibrationCh2Amp(Axis axis) { return 0; }
    uint16_t getVibrationCh3Hz(Axis axis) { return 0; }
    uint16_t getVibrationCh3Amp(Axis axis) { return 0; }

    // update internal calculations
    void process();

 private:

        void calculateElevatorForces();
        void calculateAileronForces();

        double calculateForceLiftDueToSpeed(double surfaceArea, double propWashCoeff);
private:
    Settings settings_;

    // inputs
    double elevator_ = 0.0;
    double aileron_ = 0.0;

    double airDensity_ = 0.0;
    double tas_ = 0.0;
    double gs_ = 0.0;
    double thrust_ = 0.0;
    double alphaAngleRad_ = 0.0;
    double elevatorTrim_ = 0.0;
    double pitchRate_ = 0.0;
    double cgPosFrac_ = 0.0;

    bool onGround_ = false;

    // outputs
    float fixedForce_[AxisCount] = {0.0f};
    float springForce_[AxisCount] = {0.0f};
};