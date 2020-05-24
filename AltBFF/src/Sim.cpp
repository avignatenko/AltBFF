#include "Sim.h"

#include <windows.h>
#include "FSUIPC_User64.h"

#include <spdlog/spdlog.h>

#include <set>

namespace
{
static double kPi = std::acos(-1);
}

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

    spdlog::trace("Sim Elevator Write: {}", simData_.elevator);
}
void Sim::writeAileron(double aileron)
{
    simData_.aileron = static_cast<int16_t>(aileron * 16383 / 100) * (settings_.invertFSAileron ? -1 : 1);
    simDataWriteFlags_.aileron = true;
}

double Sim::readElevator()
{
    spdlog::trace("Sim Elevator: {}", simData_.elevator);
    return simData_.elevator * 100.0 / 16383 * (settings_.invertFSElevator ? -1 : 1);
}

double Sim::readAileron()
{
    spdlog::trace("Sim Aileron: {}", simData_.aileron);
    return simData_.aileron * 100.0 / 16383 * (settings_.invertFSAileron ? -1 : 1);
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

double Sim::readAmbienAirPressure()
{
    auto pressurePfs2pa = [](double pressure) { return pressure * 47.8803; };
    return pressurePfs2pa(simData_.airPressure);

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

double Sim::readGS()
{
    return simData_.gs / 65536.0;
}

double Sim::readPitchRate()
{
    return simData_.pitchRate;
}

double Sim::readPitch()
{
    return simData_.pitch * 2.0 * kPi / 65536 / 65536;
}

double Sim::readFpm()
{
    return simData_.fpm * 60.0 * 3.28084 / 256.0;
}
double Sim::readCGPosFrac()
{
    return simData_.cgPosFrac;
}

int Sim::readEngine1RPM()
{
    return simData_.engine1RPM * 3000.0 / 16384;
}

double Sim::readEngine1Flow()
{
    return simData_.engine1Flow;
}

double Sim::readRelativeAoA()
{
    return 100 - (100.0 * simData_.relativeAoA / 32767);
}

bool Sim::readOnGround()
{
    return (simData_.onGround == 1);
}

Sim::GroundType Sim::readGroundType()
{
    if (!readOnGround()) return Sim::GroundType::NA;

    // see https://www.prepar3d.com/SDKv4/sdk/references/variables/simulation_variables.html
    std::set<int> grass = { 1, 5, 6, 8, 9 };
    std::set<int> noisy = { 2, 3, 10, 11, 12, 13, 14, 18, 19, 21, 22 };

    if (simData_.surfaceType > 23 || simData_.surfaceType < 0) // unknown?
        return Sim::GroundType::Grass;

    if (grass.find(simData_.surfaceType) != grass.end())
        return Sim::GroundType::Grass;

    if (noisy.find(simData_.surfaceType) != noisy.end())
        return Sim::GroundType::Noisy;

    return Sim::GroundType::Concrete;
}


double Sim::readCLElevatorTrim()
{
    return simData_.clElevatorTrim / 16383.0;
}

Sim::AxisControl Sim::readAxisControlState(Axis axis)
{
    switch (axis)
    {
    case Aileron: return simData_.apRollEngaged > 0 ? AxisControl::Auto : AxisControl::Manual;
    case Elevator: return simData_.apPitchEnaged > 0 ? AxisControl::Auto : AxisControl::Manual;
    }

    return AxisControl::Manual; // just in case
}

namespace
{
BOOL FSUIPC_Write_IF(DWORD dwOffset, DWORD dwSize, void* pSrce, bool write, DWORD* pdwResult)
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

        !FSUIPC_Read(0x0BB2, 2, &simData_.elevator, &dwResult) ||
        !FSUIPC_Read(0x0BB6, 2, &simData_.aileron, &dwResult) ||
        !FSUIPC_Read(0x28C0, 8, &simData_.airDensity, &dwResult) ||
        !FSUIPC_Read(0x28C8, 8, &simData_.airPressure, &dwResult) ||
        !FSUIPC_Read(0x02B8, 4, &simData_.tas, &dwResult) ||
        !FSUIPC_Read(0x2410, 8, &simData_.thrust, &dwResult) ||
        !FSUIPC_Read(0x2ED0, 8, &simData_.alpha, &dwResult) ||
        !FSUIPC_Read(0x02B4, 4, &simData_.gs, &dwResult) ||
        !FSUIPC_Read(0x0366, 2, &simData_.onGround, &dwResult) ||
        !FSUIPC_Read(0x2EF8, 8, &simData_.cgPosFrac, &dwResult) ||
        !FSUIPC_Read(0x0578, 4, &simData_.pitch, &dwResult) ||
        !FSUIPC_Read(0x30A8, 8, &simData_.pitchRate, &dwResult) ||
        !FSUIPC_Read(0x02C8, 4, &simData_.fpm, &dwResult) ||
        !FSUIPC_Read(0x0918, 8, &simData_.engine1Flow, &dwResult) ||
        !FSUIPC_Read(0x0898, 2, &simData_.engine1RPM, &dwResult) ||
        !FSUIPC_Read(0x11BE, 2, &simData_.relativeAoA, &dwResult) ||
        !FSUIPC_Read(0x31E8, 4, &simData_.surfaceType, &dwResult) ||
        !FSUIPC_Read(settings_.clElevatorTrimOffset, 2, &simData_.clElevatorTrim, &dwResult) ||
        !FSUIPC_Read(settings_.apRollEngagedOffset, 2, &simData_.apRollEngaged, &dwResult) ||
        !FSUIPC_Read(settings_.apPitchEngagedOffset, 2, &simData_.apPitchEnaged, &dwResult);

    // process
    failed = failed || !FSUIPC_Process(&dwResult);

    if (failed) spdlog::error("FSUIPC error: {}", dwResult);

    // set flags back to false (meaning we won't send values again until somebody raise the flag)
    if (!failed) simDataWriteFlags_.reset();
}