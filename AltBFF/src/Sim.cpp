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

void Sim::writeElevator(float elevator)
{
    DWORD dwResult;
    int16_t elevatorPosition = static_cast<int16_t>(elevator * 16383 / 100) * (settings_.invertFSElevator ? -1 : 1);
    FSUIPC_Write(0x0BB2, 2, &elevatorPosition, &dwResult);
}
void Sim::writeAileron(float aileron)
{
    DWORD dwResult;
    int16_t aileronPosition = static_cast<int16_t>(aileron * 16383 / 100) * (settings_.invertFSAileron ? -1 : 1);
    FSUIPC_Write(0x0BB6, 2, &aileronPosition, &dwResult);
}

double Sim::readTAS()
{
    DWORD dwResult;

    float tasInput = 0.0;
    FSUIPC_Read(0x02B8, 4, &tasInput, &dwResult);

    return tasInput / 128;
}

double Sim::readThrust()
{
    DWORD dwResult;

    float trustInput = 0.0;
    FSUIPC_Read(0x02410, 4, &trustInput, &dwResult);

    return trustInput;
}

void Sim::process()
{
    DWORD dwResult;
    FSUIPC_Process(&dwResult);
}