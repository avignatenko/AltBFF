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
    };

    Model(const Settings& settings);

    enum Axis
    {
        Elevator = 0,
        Aileron,
        AxisCount,
    };

    // inputs

    void setElevator(float elevator) { elevator_ = elevator; }
    void setAileron(float aileron) { aileron_ = aileron; }

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
    Settings settings_;

    // inputs
    float elevator_ = 0.0f;
    float aileron_ = 0.0f;

    // outputs
    float fixedForce_[AxisCount] = {0.0f};
    float springForce_[AxisCount] = {0.0f};
};