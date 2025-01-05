#pragma once

#include <cstdint>

class SimImpl
{
public:
    struct Settings
    {
        bool invertFSElevator = false;
        bool invertFSAileron = false;
        bool invertCLElevatorTrim = false;
        int propWashOffset = 0;
        int clElevatorTrimOffset = 0;
        int apRollEngagedOffset = 0;
        int apPitchEngagedOffset = 0;
        int apPitchLimitsOffset = 0;
        int clForceEnabledOffset = 0;
        int clEngageOffset = 0;
    };

    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual ~SimImpl() = default;

    [[nodiscard]] virtual bool connected() const = 0;

    struct SimData
    {
        double elevator = 0;
        double elevatorTrim = 0;
        double aileron = 0;
        int32_t tas = 0;
        int16_t relativeAoA = 0;
        double propWash = 0;
        double engine1Flow = 0.0;
        int16_t engine1RPM = 0;
        double thrust = 0.0;
        double airDensity = 0.0;
        double airPressure = 0.0;
        double pressureAltitude = 0.0;
        double alpha = 0.0;
        double pitchRate = 0.0;
        int32_t pitch = 0;
        int32_t fpm = 0;
        double cgPosFrac = 0.0;
        int32_t gs = 0;
        int16_t onGround = 0;
        int32_t surfaceType = 0;
        int16_t slewMode = 0;   // 05DC
        int16_t pauseMode = 0;  // 0264
        int8_t inmenuMode = 0;  // 3365
        double clElevatorTrim = 0;
        int8_t apRollEngaged = 0;
        int8_t apPitchEnaged = 0;
        int8_t apPitchLimits = 0;
        int8_t clForceEnabled = 0;
        int8_t clEngage = 0;
    };

    struct SimDataWriteFlags
    {
        bool elevator = false;
        bool elevatorTrim = false;
        bool aileron = false;
        bool apPitchLimits = false;
        bool clForceEnabled = false;

        void reset()
        {
            elevator = false;
            elevatorTrim = false;
            aileron = false;
            apPitchLimits = false;
            clForceEnabled = false;
        }
    };

    virtual bool process(SimData& data, SimDataWriteFlags& flags) = 0;
};
