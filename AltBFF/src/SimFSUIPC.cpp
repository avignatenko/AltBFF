#include "SimFSUIPC.h"

#include <windows.h>
#include "FSUIPC_User64.h"

#include <spdlog/spdlog.h>

namespace
{
BOOL FSUIPC_Write_IF(DWORD dwOffset, DWORD dwSize, void* pSrce, bool write, DWORD* pdwResult)
{
    if (!write) return TRUE;

    return FSUIPC_Write(dwOffset, dwSize, pSrce, pdwResult);
}
}  // namespace

SimFSUIPC::SimFSUIPC(const Settings& settings) : settings_(settings) {}

bool SimFSUIPC::connect()
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

void SimFSUIPC::disconnect()
{
    FSUIPC_Close();
    connected_ = false;
}

bool SimFSUIPC::process(SimData& data, SimDataWriteFlags& flags)
{
    if (!connected()) connect();

    // ajust values according to settings
    int16_t newElevator = data.elevator * (settings_.invertFSElevator ? -1 : 1);
    int16_t newAileron = data.aileron * (settings_.invertFSAileron ? -1 : 1);

    DWORD dwResult = FSUIPC_ERR_OK;

    BOOL failed =
        !FSUIPC_Write_IF(0x0BB2, 2, &newElevator, flags.elevator, &dwResult) ||
        !FSUIPC_Write_IF(0x0BC0, 2, &data.elevatorTrim, flags.elevatorTrim, &dwResult) ||
        !FSUIPC_Write_IF(0x0BB6, 2, &newAileron, flags.aileron, &dwResult) ||
        !FSUIPC_Write_IF(settings_.apPitchLimitsOffset, 1, &data.apPitchLimits, flags.apPitchLimits, &dwResult) ||
        !FSUIPC_Write_IF(settings_.clForceEnabledOffset, 1, &data.clForceEnabled, flags.clForceEnabled, &dwResult) ||

        !FSUIPC_Read(0x0BB2, 2, &data.elevator, &dwResult) || !FSUIPC_Read(0x0BB6, 2, &data.aileron, &dwResult) ||
        !FSUIPC_Read(0x28C0, 8, &data.airDensity, &dwResult) || !FSUIPC_Read(0x28C8, 8, &data.airPressure, &dwResult) ||
        !FSUIPC_Read(0x34B0, 8, &data.pressureAltitude, &dwResult) || !FSUIPC_Read(0x02B8, 4, &data.tas, &dwResult) ||
        !FSUIPC_Read(0x2410, 8, &data.thrust, &dwResult) || !FSUIPC_Read(0x2ED0, 8, &data.alpha, &dwResult) ||
        !FSUIPC_Read(0x02B4, 4, &data.gs, &dwResult) || !FSUIPC_Read(0x0366, 2, &data.onGround, &dwResult) ||
        !FSUIPC_Read(0x2EF8, 8, &data.cgPosFrac, &dwResult) || !FSUIPC_Read(0x0578, 4, &data.pitch, &dwResult) ||
        !FSUIPC_Read(0x30A8, 8, &data.pitchRate, &dwResult) || !FSUIPC_Read(0x02C8, 4, &data.fpm, &dwResult) ||
        !FSUIPC_Read(0x0918, 8, &data.engine1Flow, &dwResult) || !FSUIPC_Read(0x0898, 2, &data.engine1RPM, &dwResult) ||
        !FSUIPC_Read(0x11BE, 2, &data.relativeAoA, &dwResult) ||
        !FSUIPC_Read(0x31E8, 4, &data.surfaceType, &dwResult) || !FSUIPC_Read(0x05DC, 2, &data.slewMode, &dwResult) ||
        !FSUIPC_Read(0x0264, 2, &data.pauseMode, &dwResult) || !FSUIPC_Read(0x3365, 1, &data.inmenuMode, &dwResult) ||
        !FSUIPC_Read(settings_.propWashOffset, 8, &data.propWash, &dwResult) ||
        !FSUIPC_Read(settings_.clElevatorTrimOffset, 2, &data.clElevatorTrim, &dwResult) ||
        !FSUIPC_Read(settings_.apRollEngagedOffset, 1, &data.apRollEngaged, &dwResult) ||
        !FSUIPC_Read(settings_.apPitchEngagedOffset, 1, &data.apPitchEnaged, &dwResult) ||
        !FSUIPC_Read(settings_.clEngageOffset, 1, &data.clEngage, &dwResult);
    // process
    failed = failed || !FSUIPC_Process(&dwResult);

    // ajust values according to settings
    data.clElevatorTrim *= (settings_.invertCLElevatorTrim ? -1 : 1);
    data.aileron *= (settings_.invertFSAileron ? -1 : 1);
    data.elevator *= (settings_.invertFSElevator ? -1 : 1);

    if (failed)
    {
        spdlog::error("FSUIPC error: {}", dwResult);
        disconnect();  // assume FSUIPC closed, will try to re connect next time
    }
    return failed;
}
