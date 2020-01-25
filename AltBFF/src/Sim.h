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

    // read

    // m/s
    double readTAS();

    // pounds
    double readThrust();

    // range [-1, 1]
    // fixme: this can be linked directly to BFF, avoiding sim 
    double readCLElevatorTrim();

    void process();

private:

    struct
    {
        int16_t elevator = 0;
        int16_t aileron = 0;
        int32_t tas = 0.0f;
        double thrust = 0.0f;
        int16_t clElevatorTrim = 0;
    } simData_;

    bool connected_ = false;
    Settings settings_;
};