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
        Elevator,
        Aileron
    };

    int getFrictionCoeff(Axis axis)
    {
        switch (axis)
        {
        case Elevator:
            return settings_.elevatorFrictionCoeff;
        case Aileron:
            return settings_.aileronFrictionCoeff;
        default:
            return 0;
        }
    }
    int getDumpingCoeff(Axis axis)
    {
        switch (axis)
        {
        case Elevator:
            return settings_.elevatorDumpingCoeff;
        case Aileron:
            return settings_.aileronDumpingCoeff;
        default:
            return 0;
        }
    }

    float getFixedForce(Axis axis) { return 0; }
    float getSpringForce(Axis axis) { return 0; }
    uint16_t getVibrationCh1Hz(Axis axis) { return 0; }
    uint16_t getVibrationCh1Amp(Axis axis) { return 0; }
    uint16_t getVibrationCh2Hz(Axis axis) { return 0; }
    uint16_t getVibrationCh2Amp(Axis axis) { return 0; }
    uint16_t getVibrationCh3Hz(Axis axis) { return 0; }
    uint16_t getVibrationCh3Amp(Axis axis) { return 0; }

    // update internal calculations
    void process() {}

private:
    Settings settings_;
};