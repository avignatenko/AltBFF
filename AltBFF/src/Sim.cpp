#include "Sim.h"

#include <Utils/Common.h>

#ifdef _WIN32
#include "SimFSUIPC.h"
using SimImplementation = SimFSUIPC;
#else
#include "SimDummy.h"
using SimImplementation = SimDummy;
#endif

#include <spdlog/spdlog.h>

#include <set>

template <typename T>
bool isCloseToZero(T x)
{
    return std::abs(x) < std::numeric_limits<T>::epsilon();
}

template <typename T>
bool isNear(T x, T y)
{
    return isCloseToZero(x - y);
}

Sim::Sim(const SimImpl::Settings& settings) : simImpl_(std::make_unique<SimImplementation>(settings))
{
    connect();
}

Sim::~Sim()
{
    disconnect();
}

bool Sim::connect()
{
    return simImpl_->connect();
}

void Sim::disconnect()
{
    return simImpl_->disconnect();
}

bool Sim::connected() const
{
    return simImpl_->connected();
}

void Sim::writeElevator(double elevator, bool force /*= false*/)
{
    if (!force && isNear(simData_.elevator, elevator)) return;

    simData_.elevator = elevator;
    simDataWriteFlags_.elevator = true;

    spdlog::trace("Sim Elevator Write: {}", simData_.elevator);
}

void Sim::writeAileron(double aileron, bool force /*= false*/)
{
    if (!force && isNear(simData_.aileron, aileron)) return;

    simData_.aileron = aileron;
    simDataWriteFlags_.aileron = true;
}

double Sim::readElevator()
{
    spdlog::trace("Sim Elevator: {}", simData_.elevator);
    return simData_.elevator;
}

double Sim::readAileron()
{
    spdlog::trace("Sim Aileron: {}", simData_.aileron);
    return simData_.aileron;
}

void Sim::writeElevatorTrim(double trim, bool force /*= false*/)
{
    if (!force && isNear(simData_.elevatorTrim, trim)) return;

    spdlog::trace("Sim trim: {}", trim);
    simData_.elevatorTrim = trim;
    simDataWriteFlags_.elevatorTrim = true;
}

void Sim::writeAPPitchLimits(APPitchLimits limits, bool force /*= false*/)
{
    int8_t newApPitchLimits = static_cast<int8_t>(limits);

    if (!force && simData_.apPitchLimits == newApPitchLimits) return;

    simData_.apPitchLimits = newApPitchLimits;
    simDataWriteFlags_.apPitchLimits = true;
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

double Sim::readPressureAltitude()
{
    return simData_.pressureAltitude;
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

double Sim::readPropWash()
{
    return simData_.propWash;
}

bool Sim::readOnGround()
{
    return (simData_.onGround == 1);
}

bool Sim::simulationPaused()
{
    return simData_.slewMode != 0 || simData_.pauseMode != 0 || simData_.inmenuMode != 0;
}

Sim::GroundType Sim::readGroundType()
{
    if (!readOnGround()) return Sim::GroundType::NA;

    // see https://www.prepar3d.com/SDKv4/sdk/references/variables/simulation_variables.html
    std::set<int> grass = {1, 5, 6, 8, 9};
    std::set<int> noisy = {2, 3, 10, 11, 12, 13, 14, 18, 19, 21, 22};

    if (simData_.surfaceType > 23 || simData_.surfaceType < 0)  // unknown?
        return Sim::GroundType::Grass;

    if (grass.find(simData_.surfaceType) != grass.end()) return Sim::GroundType::Grass;

    if (noisy.find(simData_.surfaceType) != noisy.end()) return Sim::GroundType::Noisy;

    return Sim::GroundType::Concrete;
}

double Sim::readCLElevatorTrim()
{
    return simData_.clElevatorTrim;
}

Sim::CLEngage Sim::readCLEngage()
{
    return CLEngage(simData_.clEngage);
}

void Sim::writeCLForceEnabled(bool enabled, bool force /*= false*/)
{
    if (!force && simData_.clForceEnabled == int(enabled)) return;
    simData_.clForceEnabled = int(enabled);
    simDataWriteFlags_.clForceEnabled = true;
}

Sim::AxisControl Sim::readAxisControlState(Axis axis)
{
    switch (axis)
    {
    case Aileron:
        return simData_.apRollEngaged > 0 ? AxisControl::Auto : AxisControl::Manual;
    case Elevator:
        return simData_.apPitchEnaged > 0 ? AxisControl::Auto : AxisControl::Manual;
    }

    return AxisControl::Manual;  // just in case
}

void Sim::process()
{
    bool failed = simImpl_->process(simData_, simDataWriteFlags_);

    // set flags back to false (meaning we won't send values again until somebody raise the flag)
    if (!failed) simDataWriteFlags_.reset();
}
