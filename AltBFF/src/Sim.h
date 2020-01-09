#pragma once

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

    void writeElevator(float elevator);
    void writeAileron(float aileron);

    void process();

private:

    bool connected_ = false;
    Settings settings_;
};