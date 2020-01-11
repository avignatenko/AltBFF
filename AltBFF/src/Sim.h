#pragma once

#include <cstdint>
class Sim
{
public:
    struct Settings
    {
        bool invertFSElevator = false;
        bool invertFSAileron = false;
    };

    Sim(Settings& settings);
    ~Sim();
    bool connect();
    void disconnect();
    bool connected() { return connected_; }

    // write 

     // +/- 100%, mid = 0
    void writeElevator(float elevator);

     // +/- 100%, mid = 0
    void writeAileron(float aileron);

    // read

    // m/s
    double readTAS();

    // pounds
    double readThrust();

    void process();

private:

    struct
    {
        int16_t elevator = 0;
        int16_t aileron = 0;
        int32_t tas = 0.0f;
        double thrust = 0.0f;
    } simData_;

    bool connected_ = false;
    Settings settings_;
};