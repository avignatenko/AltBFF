#pragma once

#include <cstdint>
class Sim
{
public:
    struct Settings
    {
        bool invertFSElevator = false;
        bool invertFSAileron = false;
        int clElevatorTrimOffset = 0;
        int apRollEngagedOffset = 0;
        int apPitchEngagedOffset = 0;
        int apPitchLimitsOffset = 0;
    };

    Sim(Settings& settings);
    ~Sim();
    bool connect();
    void disconnect();
    bool connected() { return connected_; }

    // write

    // +/- 100%, mid = 0
    void writeElevator(double elevator);

    // +/- 100%, mid = 0
    void writeAileron(double aileron);

    // +/- 1, mid = 0
    void writeElevatorTrim(double trim);

    enum class APPitchLimits
    {
        TrimOk,
        TrimUp,
        TrimUpMore,
        TrimDown,
        TrimDownMore
    };

    void writeAPPitchLimits(APPitchLimits limits);

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

    // range [-1, 1]
    // fixme: this can be linked directly to BFF, avoiding sim
    double readCLElevatorTrim();

    // ap

    enum Axis
    {
        Elevator = 0,
        Aileron,
        AxisCount,
    };

    enum class AxisControl
    {
        Manual,
        Auto
    };

    AxisControl readAxisControlState(Axis axis);

    void process();

private:
    struct
    {
        int16_t elevator = 0;
        int16_t elevatorTrim = 0;
        int16_t aileron = 0;
        int32_t tas = 0;
        int16_t relativeAoA = 0;
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
        int16_t clElevatorTrim = 0;
        int8_t apRollEngaged = 0;
        int8_t apPitchEnaged = 0;
        int8_t apPitchLimits;
    } simData_;

    struct
    {
        bool elevator = false;
        bool elevatorTrim = false;
        bool aileron = false;
        bool apPitchLimits = false;

        void reset()
        {
            elevator = false;
            elevatorTrim = false;
            aileron = false;
            apPitchLimits = false;
        }

    } simDataWriteFlags_;

    bool connected_ = false;
    Settings settings_;
};