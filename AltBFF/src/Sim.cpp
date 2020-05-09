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
    simDataWriteFlags_.elevator = true;
}
void Sim::writeAileron(double aileron)
{
    simData_.aileron = static_cast<int16_t>(aileron * 16383 / 100) * (settings_.invertFSAileron ? -1 : 1);
    simDataWriteFlags_.aileron = true;
}

void Sim::writeElevatorTrim(double trim)
{
    simData_.elevatorTrim = static_cast<int16_t>(trim * 16383);
    simDataWriteFlags_.elevatorTrim = true;
}

double Sim::readAmbientAirDensity()
{
    auto densityslugs2kg = [](double density) { return density * 515.38; };
    return densityslugs2kg(simData_.airDensity);
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

double Sim::readAlpha()
{
    return simData_.alpha;
}

double Sim::readCLElevatorTrim()
{
    return simData_.clElevatorTrim / 16383.0;
}

namespace
{
BOOL FSUIPC_Write_IF(DWORD dwOffset, DWORD dwSize, void *pSrce, bool write, DWORD *pdwResult)
{
    if (!write) return TRUE;

    return FSUIPC_Write(dwOffset, dwSize, pSrce, pdwResult);
}
}  // namespace

void Sim::process()
{
    DWORD dwResult = FSUIPC_ERR_OK;

    BOOL failed = !FSUIPC_Write_IF(0x0BB2, 2, &simData_.elevator, simDataWriteFlags_.elevator, &dwResult) ||
                  !FSUIPC_Write_IF(0x0BC0, 2, &simData_.elevatorTrim, simDataWriteFlags_.elevatorTrim, &dwResult) ||
                  !FSUIPC_Write_IF(0x0BB6, 2, &simData_.aileron, simDataWriteFlags_.aileron, &dwResult) ||

                  !FSUIPC_Read(0x28C0, 8, &simData_.airDensity, &dwResult) ||
                  !FSUIPC_Read(0x02B8, 4, &simData_.tas, &dwResult) ||
                  !FSUIPC_Read(0x2410, 8, &simData_.thrust, &dwResult) ||
                  !FSUIPC_Read(0x2ED0, 8, &simData_.alpha, &dwResult) ||
                  !FSUIPC_Read(settings_.clElevatorTrimOffset, 2, &simData_.clElevatorTrim, &dwResult);

    // process
    failed = failed || !FSUIPC_Process(&dwResult);

    if (failed) spdlog::error("FSUIPC error: {}", dwResult);

    // set flags back to false (meaning we won't send values again until somebody raise the flag)
    if (!failed) simDataWriteFlags_.reset();
}