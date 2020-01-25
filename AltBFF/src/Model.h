#pragma once

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

        double elevatorArea = 0.0;
        double clExponent = 0.0;
        double propWashCoeff = 0.0;
        double maxElevatorLift = 0.0;
        double maxElevatorAngleRadians = 0.0;
    };

    Model(const Settings& settings);

    enum Axis
    {
        Elevator = 0,
        Aileron,
        AxisCount,
    };

    // inputs

    // +/- 100%, mid = 0
    void setElevator(double elevator) { elevator_ = elevator; }

    // +/- 100%, mid = 0
    void setAileron(double aileron) { aileron_ = aileron; }

    // tas in ms/sec
    void setTAS(double tas) { tas_ = tas; }

    // thrust in pounds
    void setThrust(double thrust) { thrust_ = thrust; }

    void setCLElevatorTrim(double clElevatorTrim) { clElevatorTrim_ = clElevatorTrim; }
    
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
    
private:
    Settings settings_;

    // inputs
    double elevator_ = 0.0;
    double aileron_ = 0.0;
    double tas_ = 0.0;
    double thrust_ = 0.0;
    double clElevatorTrim_ = 0.0;

    // outputs
    float fixedForce_[AxisCount] = {0.0f};
    float springForce_[AxisCount] = {0.0f};
};