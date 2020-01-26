#include "Sim.h"

#include <windows.h>
#include "FSUIPC_User64.h"

#include <spdlog/spdlog.h>

Sim::Sim(Settings& settings) : settings_(settings)
{
    connect();
}

Sim::~Sim()
{
    disconnect();
}

bool Sim::connect()
{
    DWORD dwResult;
    bool fsuipcPresent = FSUIPC_Open(SIM_ANY, &dwResult);
    if (fsuipcPresent)
        spdlog::info("FSUIPC found");
    else
        spdlog::error("FSUIPC not found (error {}), continue in test mode", dwResult);

    connected_ = fsuipcPresent;

    return fsuipcPresent;
}

void Sim::disconnect()
{
    FSUIPC_Close();
    connected_ = false;
}

void Sim::writeElevator(double elevator)
{
    simData_.elevator = static_cast<int16_t>(elevator * 16383 / 100) * (settings_.invertFSElevator ? -1 : 1);
}
void Sim::writeAileron(double aileron)
{
    simData_.aileron = static_cast<int16_t>(aileron * 16383 / 100) * (settings_.invertFSAileron ? -1 : 1);
}

double Sim::readTAS()
{
    auto kn2mps = [](double kn) { return kn * 0.515; };
    return kn2mps(simData_.tas / 128);
}

double Sim::readThrust()
{
    return simData_.thrust;
}

double Sim::readCLElevatorTrim() 
{
    return simData_.clElevatorTrim / 16383.0;
}

void Sim::process()
{
    DWORD dwResult;

    BOOL failed = !FSUIPC_Write(0x0BB2, 2, &simData_.elevator, &dwResult) ||
                  !FSUIPC_Write(0x0BB6, 2, &simData_.aileron, &dwResult) ||
                  !FSUIPC_Read(0x02B8, 4, &simData_.tas, &dwResult) ||
                  !FSUIPC_Read(0x2410, 8, &simData_.thrust, &dwResult);

    if (settings_.clElevatorTrimOffset > 0)
        failed = failed || !FSUIPC_Read(settings_.clElevatorTrimOffset, 2, &simData_.clElevatorTrim, &dwResult);

    // process
    failed = failed || !FSUIPC_Process(&dwResult);

    spdlog::debug("tas: {}, thrust: {}", simData_.tas, simData_.thrust);

    if (failed) spdlog::error("FSUIPC error: {}", dwResult);
}