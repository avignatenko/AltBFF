#include "Model.h"

#include <Utils/Common.h>

#include <algorithm>
#include <cmath>

Model::Model(const Settings& settings)
{
    setSettings(settings);
}

void Model::setSettings(const Settings& settings)
{
    settings_ = settings;
    spdlog::trace("Settings set");
    spdlog::trace("Settings::calculatePropWash: {}", settings_.calculatePropWash);
}

int Model::getFrictionCoeff(Axis axis) const
{
    switch (axis)
    {
    case Elevator:
        return settings_.hardware.elevatorFrictionCoeff;
    case Aileron:
        return settings_.hardware.aileronFrictionCoeff;
    default:
        return 0;
    }
}

int Model::getDumpingCoeff(Axis axis) const
{
    switch (axis)
    {
    case Elevator:
        return settings_.hardware.elevatorDumpingCoeff;
    case Aileron:
        return settings_.hardware.aileronDumpingCoeff;
    default:
        return 0;
    }
}

int Model::getPositionFollowingP(Axis axis) const
{
    switch (axis)
    {
    case Elevator:
        return settings_.hardware.elevatorPositionFollowingP;
    case Aileron:
        return settings_.hardware.aileronPositionFollowingP;
    default:
        return 0;
    }
}

int Model::getPositionFollowingI(Axis axis) const
{
    switch (axis)
    {
    case Elevator:
        return settings_.hardware.elevatorPositionFollowingI;
    case Aileron:
        return settings_.hardware.aileronPositionFollowingI;
    default:
        return 0;
    }
}

int Model::getPositionFollowingD(Axis axis) const
{
    switch (axis)
    {
    case Elevator:
        return settings_.hardware.elevatorPositionFollowingD;
    case Aileron:
        return settings_.hardware.aileronPositionFollowingD;
    default:
        return 0;
    }
}
// update internal calculations

void Model::process()
{
    calculateElevatorForces();
    calculateAileronForces();
    calculateEngineVibrations();
    calculateStallVibrations();
    calculateRunwayVibrations();
}

double fixAlphaWrap(double alpha)
{
    if (alpha > kPi / 2) alpha = -kPi + alpha;
    if (alpha < -kPi / 2) alpha = kPi + alpha;
    return std::clamp(alpha, -kPi / 2.0, kPi / 2);  // limit just in case..
}

double scaleAlpha(double alpha, double scaleThesholdKn, double gs)
{
    auto ms2kn = [](double ms) { return ms * 1.944; };
    double scaleCoeff = 1.0;
    if (scaleThesholdKn > 0) scaleCoeff = std::clamp(ms2kn(std::abs(gs)), 0.0, scaleThesholdKn) / scaleThesholdKn;

    return alpha * scaleCoeff;
}

double Model::calculateForceLiftDueToSpeed(double surfaceArea, double propWashCoeff)
{
    double airSpeed2 = 0;
    if (settings_.calculatePropWash)
    {
        // test advanced prop wash calculation (for single prop GA)
        // https://www.grc.nasa.gov/www/k-12/airplane/propth.html
        double thrustN = std::max(thrust_, 0.0) * 4.45;       // pounds to newton
        const double propArea = 3.14 / 4 * std::pow(1.8, 2);  // prop diameter roughly 1.8m

        airSpeed2 = std::sqrt(thrustN / (.5 * airDensity_ * propArea) * std::pow(propWashCoeff, 2.0) + tas_ * tas_);
    }
    else
    {
        airSpeed2 = propWash_ * propWashCoeff + tas_;
    }

    spdlog::debug("Airspeeds tas: {}, propwash_new: {}", tas_, airSpeed2);

    // Calculate lift force for elevator
    // From lift equation: L = Cl * pho * V^2 * A / 2
    // ((https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/lift-equation/) note: I sue kBaseAirDensity because
    // it's already accounted in TAS
    double flElevatorDueToSpeed =
        surfaceArea * kBaseAirDensity / 2.0 * std::pow(std::abs(airSpeed2), settings_.clExponent);

    return flElevatorDueToSpeed;
}

/*
void Model::calculateElevatorForces2()
{
    auto forceFromSpeed = [this] ()
    {
        double clCoeffElevator = settings_.maxElevatorLift / settings_.maxElevatorAngleRadians; // 2.0 * kPi;
        double force = clCoeffElevator * calculateForceLiftDueToSpeed(settings_.elevatorArea,
settings_.propWashElevatorCoeff) / 100.0; return force;
    };

    double alpha = fixAlphaWrap(alphaAngleRad_);
    double forceElevatorProportional = settings_.maxElevatorAngleRadians * forceFromSpeed();
    double forceElevator = (elevator_  / 100.0) * forceElevatorProportional;
    double forceElevatorTrim = elevatorTrim_ * forceElevatorProportional * settings_.elevatorTrimGain;
    double forceAlpha = alpha * forceFromSpeed();
    double force = (-1.0) * forceElevator +  forceAlpha + forceElevatorTrim;

    // cl forces

    fixedForce_[Elevator] = forceAlpha + forceElevatorTrim;
    springForce_[Elevator] = forceElevatorProportional / 100.0;;

    //double totalForce = -springForce_[Elevator] * elevator_ + fixedForce_[Elevator];

    // zero force point [-100, 100]
    //elevator = 100.0 * (alpha / settings_.maxElevatorAngleRadians + elevatorTrim_  * settings_.elevatorTrimGain);

}
*/

void Model::calculateElevatorForces()
{
    double clCoeffElevator = settings_.maxElevatorLift / settings_.maxElevatorAngleRadians;

    double flElevatorDueToSpeed =
        calculateForceLiftDueToSpeed(settings_.elevatorArea, settings_.propWashElevatorCoeff) / 100.0;

    // spring force measures from max elevator force
    double clElevatorMax = clCoeffElevator * settings_.maxElevatorAngleRadians;
    double flElevatorSpring = clElevatorMax * flElevatorDueToSpeed / 100.0;

    // trim works same as elevator, but with gain (effectiveness)
    auto axisOffset = [](double trim, double neutralPos)
    {
        // offset and normalize
        double axisOffset = (trim - neutralPos) / (1 + std::abs(neutralPos));
        return axisOffset;
    };

    double elevatorTrimOffset = axisOffset(elevatorTrim_, settings_.elevatorTrimNeutralPos);
    double elevatorTrimDeflectionAngleRad = elevatorTrimOffset * settings_.maxElevatorAngleRadians;
    double clElevatorTrim = clCoeffElevator * elevatorTrimDeflectionAngleRad;
    double fElevatorTrim = clElevatorTrim * flElevatorDueToSpeed * settings_.elevatorTrimGain;

    // alpha works same as elevator, but using body alpha

    // fix alpha > pi/2
    double alphaMod = fixAlphaWrap(alphaAngleRad_);

    // fix alpha on ground slow speed)
    alphaMod = scaleAlpha(alphaMod, settings_.elevatorAlphaScaleSpeedKn, gs_);

    // double rTail = std::abs(settings_.hTailPosLon) - (cgPosFrac_ * settings_.wingRootChord);
    // double alphaDueToPitch = (pitchRate_ * rTail) / clampMin(tas_, 0.001) * settings_.elevatorPRGain;
    // spdlog::trace("rTail: {}, hTailPosLon: {}, cgPosFrac: {}, chord: {}", rTail, settings_.hTailPosLon, cgPosFrac_,
    // settings_.wingRootChord);

    double clElevatorAlpha = std::clamp(clCoeffElevator * alphaMod, -clElevatorMax, clElevatorMax);
    double fElevatorAlpha = -1.0 * clElevatorAlpha * flElevatorDueToSpeed * settings_.elevatorAlphaGain;

    spdlog::debug("Alpha force: {} (gs = {})", fElevatorAlpha, gs_);
    spdlog::debug("Trim force: {}", fElevatorTrim);
    spdlog::trace("Trim pos: {} -> offseted: {}", elevatorTrim_, elevatorTrimOffset);

    double fElevatorWeight = 0.0;  // todo

    double flElevatorFixed = fElevatorAlpha + fElevatorTrim + fElevatorWeight;

    //  update elevator

    // note: average fixed force to avoid possible oscillations
    fixedForceAccum_[Elevator].addSample(flElevatorFixed);
    fixedForce_[Elevator] = fixedForceAccum_[Elevator].get();
    springForce_[Elevator] = flElevatorSpring;

    spdlog::debug("El Spring force: {}, El Fixed Force average: {}, Src Fixed Force: {}", springForce_[Elevator],
                  fixedForce_[Elevator], flElevatorFixed);
}

double Model::getTotalForce(Axis axis) const
{
    double axisPos = 0.0;
    switch (axis)
    {
    case Elevator:
        axisPos = elevator_;
        break;
    case Aileron:
        axisPos = aileron_;
        break;
    }

    return -springForce_[axis] * axisPos + fixedForce_[axis];
}

void Model::calculateAileronForces()
{
    double clCoeffAileron = settings_.maxAileronLift / settings_.maxAileronAngleRadians;

    // Calculate lift force for aileron
    double flAileronDueToSpeed =
        calculateForceLiftDueToSpeed(settings_.aileronArea, settings_.propWashAileronCoeff) / 100.0;

    // spring force measures from max aileron force
    double clAileronMax = clCoeffAileron * settings_.maxAileronAngleRadians;
    double flAileronSpring = clAileronMax * flAileronDueToSpeed / 100.0;

    double fAlphaAileron = 0.0;   // todo
    double fAileronTrim = 0.0;    // todo
    double fAileronWeight = 0.0;  // todo

    double flAileronFixed = fAlphaAileron + fAileronTrim + fAileronWeight;

    spdlog::debug("Ail Spring force: {}, Ail Fixed Force: {}", flAileronSpring, flAileronFixed);

    //  update aileron forces

    // note: average fixed force to avoid possible oscillations
    fixedForceAccum_[Aileron].addSample(flAileronFixed);
    fixedForce_[Aileron] = fixedForceAccum_[Aileron].get();
    springForce_[Aileron] = flAileronSpring;
}

void Model::calculateEngineVibrations()
{
    const double engine1CPS = engine1RPM_ / 60.0;

    const double vibrationAirGain = (onGround_ ? 1.0 : settings_.engineVibAirGain);

    // elevator
    double elevatorVibIntensity = (engine1Flow_ * settings_.elevatorEngineFlowGain * vibrationAirGain) / 100.0;
    double elevatorVibCPS = engine1CPS * settings_.elevatorEngineFreqGain;

    if (elevatorVibCPS < settings_.elevatorEngineFreqMin) elevatorVibCPS = settings_.elevatorEngineFreqMin;

    // engine vibrations go to channel 1
    vibrationsEngine_[Elevator].amp = static_cast<uint16_t>(elevatorVibIntensity);
    vibrationsEngine_[Elevator].hz = static_cast<uint16_t>(elevatorVibCPS);

    // aileron

    double aileronVibIntensity = (engine1Flow_ * settings_.aileronEngineFlowGain * vibrationAirGain) / 100.0;
    double aileronVibCPS = engine1CPS * settings_.aileronEngineFreqGain;

    if (aileronVibCPS < settings_.aileronEngineFreqMin) aileronVibCPS = settings_.aileronEngineFreqMin;

    vibrationsEngine_[Aileron].amp = static_cast<uint16_t>(aileronVibIntensity);
    vibrationsEngine_[Aileron].hz = static_cast<uint16_t>(aileronVibCPS);
}

void Model::calculateStallVibrations()
{
    // elevator
    double elevatorAngleFactor = std::clamp((std::abs(relativeAoA_ / 100.0) - 0.5) * 2.0, 0.0, 1.0);
    double elevatorSpeedFactor = std::abs(std::pow(tas_, 2)) / 10000.0;

    double elevatorVibIntensity =
        100 * (settings_.elevatorVibStallGain / 10.0) * elevatorAngleFactor * elevatorSpeedFactor;
    double elevatorVibCPS = settings_.elevatorVibStalFreq;

    vibrationsStall_[Elevator].amp = static_cast<uint16_t>(elevatorVibIntensity);
    vibrationsStall_[Elevator].hz = static_cast<uint16_t>(elevatorVibCPS);

    // aileron
    double aileronAngleFactor = std::clamp((std::abs(relativeAoA_ / 100.0) - 0.5) * 2.0, 0.0, 1.0);
    double aileronSpeedFactor = std::abs(std::pow(tas_, 2)) / 10000.0;

    double aileronVibIntensity = 100 * (settings_.aileronVibStallGain / 10.0) * aileronAngleFactor * aileronSpeedFactor;
    double aileronVibCPS = settings_.aileronVibStalFreq;

    vibrationsStall_[Aileron].amp = static_cast<uint16_t>(aileronVibIntensity);
    vibrationsStall_[Aileron].hz = static_cast<uint16_t>(aileronVibCPS);
}

void Model::calculateRunwayVibrations()
{
    auto calculateRunwayVibrationsForAxis = [this](double runwayAmp, double runwayCPS)
    {
        if (!onGround_) return std::pair<double, double>(0.0, 0.0);

        const double takeoffSpeedMps = 50;  // empirical

        double scFact = 1.0;
        if (std::abs(gs_ / takeoffSpeedMps) < 0.1)
            scFact = 0.5 * std::abs(gs_ / (takeoffSpeedMps * 0.1));
        else if (std::abs(gs_ / takeoffSpeedMps) < 1.0)
            scFact = 0.5 + (0.5 * ((std::abs(gs_) - (0.2 * takeoffSpeedMps)) / takeoffSpeedMps));

        double AmpRunway = std::clamp(runwayAmp * scFact, 0.0, runwayAmp);

        if (groundType_ == GroundType::Concrete) AmpRunway /= 4.0;
        if (groundType_ == GroundType::Grass) AmpRunway /= 1.5;

        double scFactCPS = std::clamp(0.5 + std::abs(0.5 * gs_ / takeoffSpeedMps), 0.0, 1.0);
        double CPSRunway = std::clamp(runwayCPS * scFactCPS, 0.0, runwayCPS);

        return std::pair<double, double>(AmpRunway, CPSRunway);
    };

    auto [ampEl, cpsEl] =
        calculateRunwayVibrationsForAxis(settings_.elevatorVibRunwayGain, settings_.elevatorVibRunwayFreq);

    vibrationsRunway_[Elevator].amp = static_cast<uint16_t>(ampEl);
    vibrationsRunway_[Elevator].hz = static_cast<uint16_t>(cpsEl);

    auto [ampAil, cpsAil] =
        calculateRunwayVibrationsForAxis(settings_.aileronVibRunwayGain, settings_.aileronVibRunwayFreq);

    vibrationsRunway_[Aileron].amp = static_cast<uint16_t>(ampAil);
    vibrationsRunway_[Aileron].hz = static_cast<uint16_t>(cpsAil);
}
