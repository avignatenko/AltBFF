#pragma once

#include "SimImpl.h"

#include <memory>

class Sim
{
public:
    explicit Sim(const SimImpl::Settings& settings);
    ~Sim();

    bool connect();
    void disconnect();
    [[nodiscard]] bool connected() const;

    // write

    // +/- 100%, mid = 0
    void writeElevator(double elevator, bool force = false);

    // +/- 100%, mid = 0
    void writeAileron(double aileron, bool force = false);

    // +/- 1, mid = 0
    void writeElevatorTrim(double trim, bool force = false);

    enum class APPitchLimits
    {
        TrimOk,
        TrimUp,
        TrimUpMore,
        TrimDown,
        TrimDownMore
    };

    void writeAPPitchLimits(APPitchLimits limits, bool force = false);

    // read

    // +/- 100%, mid = 0
    double readElevator();

    // +/- 100%, mid = 0
    double readAileron();

    // kg / m^3
    double readAmbientAirDensity();

    // Pa
    double readAmbienAirPressure();

    // meters
    double readPressureAltitude();

    // m/s
    double readTAS();

    // pounds
    double readThrust();

    // incidence "alpha" in radians
    double readAlpha();

    // true if on ground
    bool readOnGround();

    /// any kind of simulation pause (slew, in-menu, pause)
    bool simulationPaused();

    enum class GroundType
    {
        NA,
        Concrete,
        Grass,
        Noisy
    };

    GroundType readGroundType();

    // m/sec
    double readGS();

    // rad/sec
    double readPitchRate();

    // rad
    double readPitch();

    // fpm, +ve up, -ve down
    double readFpm();

    double readCGPosFrac();

    int readEngine1RPM();

    // Pounds per Hour
    double readEngine1Flow();

    // [0, 100]
    double readRelativeAoA();

    // m/s
    double readPropWash();

    // range [-1, 1]
    // fixme: this can be linked directly to BFF, avoiding sim
    double readCLElevatorTrim();

    enum class CLEngage
    {
        NoChange = 0,
        Engage = 1,
        Disengage = 2
    };

    CLEngage readCLEngage();

    void writeCLForceEnabled(bool enabled, bool force = false);

    // ap

    enum Axis
    {
        Elevator = 0,
        Aileron
    };

    constexpr static int AxisCount = 2;

    enum class AxisControl
    {
        Manual,
        Auto
    };

    AxisControl readAxisControlState(Axis axis);

    void process();

private:
    SimImpl::SimData simData_;
    SimImpl::SimDataWriteFlags simDataWriteFlags_;
    std::unique_ptr<SimImpl> simImpl_;
};
