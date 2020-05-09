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

    // read

    // kg / m^3 
    double readAmbientAirDensity();
    
    // m/s
    double readTAS();

    // pounds
    double readThrust();

    // incidence "alpha" in radians
    double readAlpha();

    // true if on ground
    bool readOnGround();

    // m/sec
    double readGS();

    // rad/sec
    double readPitchRate();

    double readCGPosFrac();

    // range [-1, 1]
    // fixme: this can be linked directly to BFF, avoiding sim
    double readCLElevatorTrim();

    void process();

private:
    struct
    {
        int16_t elevator = 0;
        int16_t elevatorTrim = 0;
        int16_t aileron = 0;
        int32_t tas = 0;
        double thrust = 0.0;
        double airDensity = 0.0;
        double alpha = 0.0;
        double pitchRate = 0.0;
        double cgPosFrac = 0.0;
        int32_t gs = 0;
        int16_t onGround = 0;
        int16_t clElevatorTrim = 0;
    } simData_;

    struct
    {
        bool elevator = false;
        bool elevatorTrim = false;
        bool aileron = false;

        void reset()
        {
            elevator = false;
            elevatorTrim = false;
            aileron = false;
        }
    } simDataWriteFlags_;

    bool connected_ = false;
    Settings settings_;
};