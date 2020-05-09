#include "Model.h"

#include <cmath>



namespace
{
const double kBaseAirDensity = 1.2; // at msl, used with tas

inline double clampMin(double val, double min) { return val < min ? min : val; }

}

Model::Model(const Settings& settings) 
{
    setSettings(settings);
}


void Model::setSettings(const Settings& settings)
{
    settings_ = settings;
}
int Model::getFrictionCoeff(Axis axis)
{
    switch (axis)
    {
    case Elevator:
        return settings_.elevatorFrictionCoeff;
    case Aileron:
        return settings_.aileronFrictionCoeff;
    default:
        return 0;
    }
}

int Model::getDumpingCoeff(Axis axis)
{
    switch (axis)
    {
    case Elevator:
        return settings_.elevatorDumpingCoeff;
    case Aileron:
        return settings_.aileronDumpingCoeff;
    default:
        return 0;
    }
}

// update internal calculations

void Model::process()
{
    calculateElevatorForces();
    calculateAileronForces();
}

double Model::calculateForceLiftDueToSpeed(double surfaceArea, double propWashCoeff)
{
    // test advanced prop wash calculation (for single prop GA)
    // https://www.grc.nasa.gov/www/k-12/airplane/propth.html
    double thrustN = clampMin(thrust_, 0.0) * 4.45; // pounds to newton
    const double propArea = 3.14 / 4 * std::pow(1.8, 2); // prop diameter roughly 1.8m
    double airSpeed2 = std::sqrt(thrustN / (.5 * airDensity_ * propArea) * std::pow(propWashCoeff, 2.0) + tas_ * tas_);

    // add prop wash airspeed to normal airspeed (simple model)
    double propWashAirSpeed = std::pow(clampMin(thrust_, 0.0) / 10.0, 0.8);
    double airSpeed = tas_ + (propWashAirSpeed * propWashCoeff);

    spdlog::debug("Airspeeds tas: {}, propwash_old: {}, propwash_new: {}", tas_, airSpeed, airSpeed2);

    // Calculate lift force for elevator
   // From lift equation: L = Cl * pho * V^2 * A / 2 ((https://www.grc.nasa.gov/www/k-12/airplane/lifteq.html)
    // note: I sue kBaseAirDensity because it's already accounted in TAS
    double flElevatorDueToSpeed =
        surfaceArea / 100.0 * kBaseAirDensity / 2.0 * std::pow(std::abs(airSpeed2), settings_.clExponent);

    return flElevatorDueToSpeed;
}

void Model::calculateElevatorForces()
{
    double clCoeffElevator = settings_.maxElevatorLift / settings_.maxElevatorAngleRadians;
 
    double flElevatorDueToSpeed = calculateForceLiftDueToSpeed(settings_.elevatorArea, settings_.propWashElevatorCoeff);

    // spring force measures from max elevator force
    double clElevatorMax = clCoeffElevator * settings_.maxElevatorAngleRadians;
    double flElevatorSpring = clElevatorMax * flElevatorDueToSpeed / 255;

    // trim works same as elevator, but with gain (effectiveness)
    double elevatorTrimDeflectionAngleRad = elevatorTrim_ * settings_.maxElevatorAngleRadians;
    double clElevatorTrim = clCoeffElevator * elevatorTrimDeflectionAngleRad;
    double fElevatorTrim = clElevatorTrim * flElevatorDueToSpeed * settings_.elevatorTrimGain;

    // alpha works same as elevator, but using body alpha

    // fix alpha on ground (low speed)
    auto ms2kn = [](double ms) { return ms * 1.944; };
    const double scaleThesholdKn = 20;
    double scaleCoeff = std::clamp(ms2kn(std::abs(gs_)), 0.0, scaleThesholdKn) / scaleThesholdKn;

    double rTail = std::abs(settings_.hTailPosLon) - (cgPosFrac_ * settings_.wingRootChord);
    double alphaDueToPitch = (pitchRate_ * rTail) / clampMin(tas_, 0.001) * settings_.elevatorPRGain;

    spdlog::trace("rTail: {}, hTailPosLon: {}, cgPosFrac: {}, chord: {}", rTail, settings_.hTailPosLon, cgPosFrac_, settings_.wingRootChord);

    double alphaAngleRadScaled = (alphaAngleRad_  + alphaDueToPitch) * scaleCoeff;
    
    double clElevatorAlpha =  std::clamp(clCoeffElevator * alphaAngleRadScaled, -clElevatorMax, clElevatorMax);
    double fElevatorAlpha = -1.0 * clElevatorAlpha * flElevatorDueToSpeed * settings_.elevatorAlphaGain;

    spdlog::debug("Alpha force: {} (alpha_pitch = {}, gs = {}, scale = {})", fElevatorAlpha, alphaDueToPitch, gs_, scaleCoeff);

    double fElevatorWeight = 0.0; // todo

    double flElevatorFixed = fElevatorAlpha + fElevatorTrim + fElevatorWeight;

    spdlog::debug("Trim force: {}", fElevatorTrim);

    spdlog::debug("El Spring force: {}, El Fixed Force: {}", flElevatorSpring, flElevatorFixed);
    //  update elevator
    fixedForce_[Elevator] = flElevatorFixed;
    springForce_[Elevator] = flElevatorSpring;
}

void Model::calculateAileronForces()
{
    double clCoeffAileron = settings_.maxAileronLift / settings_.maxAileronAngleRadians;

    // Calculate lift force for aileron
    // From lift equation: L = Cl * pho * V^2 * A / 2 ((https://www.grc.nasa.gov/www/k-12/airplane/lifteq.html)
    double flAileronDueToSpeed = calculateForceLiftDueToSpeed(settings_.aileronArea, settings_.propWashAileronCoeff);

    // spring force measures from max aileron force
    double clAileronMax = clCoeffAileron * settings_.maxAileronAngleRadians;
    double flAileronSpring = clAileronMax * flAileronDueToSpeed / 255;

    double fAlphaAileron = 0.0; // todo
    double fAileronTrim = 0.0; // todo
    double fAileronWeight = 0.0; // todo

    double flAileronFixed = fAlphaAileron + fAileronTrim + fAileronWeight;

    spdlog::debug("Ail Spring force: {}, Ail Fixed Force: {}", flAileronSpring, flAileronFixed);

    //  update elevator
    fixedForce_[Aileron] = flAileronFixed;
    springForce_[Aileron] = flAileronSpring;
}