#include "Model.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <optional>
#include <chrono>

namespace
{
const double kBaseAirDensity = 1.2; // at msl, used with tas

inline double clampMin(double val, double min) { return val < min ? min : val; }

static double kPi = std::acos(-1);
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
    calculateEngineVibrations();
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
    spdlog::debug("Trim force: {}", fElevatorTrim);

    double fElevatorWeight = 0.0; // todo

    double flElevatorFixed = fElevatorAlpha + fElevatorTrim + fElevatorWeight;
   
    //  update elevator

    // note: average fixed force to avoid possible oscillations
    fixedForceAccum_[Elevator](flElevatorFixed);
    fixedForce_[Elevator] = boost::accumulators::rolling_mean(fixedForceAccum_[Elevator]);
    springForce_[Elevator] = flElevatorSpring;

    spdlog::debug("El Spring force: {}, El Fixed Force average: {}, Src Fixed Force: {}", 
        springForce_[Elevator], 
        fixedForce_[Elevator], 
        flElevatorFixed);

}

void Model::calculateAileronForces()
{
    double clCoeffAileron = settings_.maxAileronLift / settings_.maxAileronAngleRadians;

    // Calculate lift force for aileron
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

/*
class Vibrator
{
public:

    // return amp, cps
    std::tuple<double, double> calculate(int n, int cps, double Intens, double Rel_Angle)
    {
        if (!prevTime_) prevTime_ = std::chrono::high_resolution_clock::now();

        auto now = std::chrono::high_resolution_clock::now();

        auto Delta_T = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(now - prevTime_.value())).count();
   
        double Speed_Factor = 1.0;
        double Angle_Factor = std::clamp((std::abs(Rel_Angle) - 0.5) * 2, 0.0, 1.0);

        double Period = 1.0 / cps;
        Angle_1 += ((2.0 * kPi) / Period * Delta_T);
        Angle_1 = std::fmod(Angle_1, 2 * kPi);
        
        double Amp1 = std::sin(Angle_1);

        double Period2 = 1.0 / cps * 1.333;
        Angle_2 += ((2.0 * kPi) / Period2 * Delta_T);
        Angle_2 = std::fmod(Angle_2, 2 * kPi);

        double Amp2 = std::sin(Angle_2);

        double Period3 = 1.0 / cps * 1.666;
        Angle_3 += ((2.0 * kPi) / Period3 * Delta_T);
        Angle_3 = std::fmod(Angle_3, 2 * kPi);
        double Amp3 = std::sin(Angle_3);

        double Amp = Amp1;
        if (n == 2) Amp += Amp2;
        if (n == 3) Amp += Amp3;

        Amp *= (Intens * Angle_Factor * Speed_Factor);
  
        double Amp_Eng = Intens * Angle_Factor * Speed_Factor;
        double CPS_Eng = cps;

        return { Amp_Eng, CPS_Eng };
    }
private:

    double Angle_1 = 0;
    double Angle_2 = 0;
    double Angle_3 = 0;

    std::optional<std::chrono::time_point<std::chrono::high_resolution_clock>> prevTime_ = std::nullopt;
};
*/
void Model::calculateEngineVibrations()
{
    const int kEngineVibrationsChannel = 1;
    const double engine1CPS = engine1RPM_ / 60.0;

    // elevator
    double elevatorVibIntensity =  (engine1Flow_ * settings_.elevatorEngineFlowGain) / 100.0;
    double elevatorVibCPS = engine1CPS * settings_.elevatorEngineFreqGain;
    
    if (elevatorVibCPS < settings_.elevatorEngineFreqMin)
        elevatorVibCPS = settings_.elevatorEngineFreqMin;
    
    // engine vibrations go to channel 1
    vibrationsAmp[kEngineVibrationsChannel][Elevator] = static_cast<uint16_t>(elevatorVibIntensity);
    vibrationsHz[kEngineVibrationsChannel][Elevator] = static_cast<uint16_t>(elevatorVibCPS);

    // aileron

    double aileronVibIntensity = (engine1Flow_ * settings_.aileronEngineFlowGain) / 100.0;
    double aileronVibCPS = engine1CPS * settings_.aileronEngineFreqGain;

    if (aileronVibCPS < settings_.aileronEngineFreqMin)
        aileronVibCPS = settings_.aileronEngineFreqMin;

    // engine vibrations go to channel 1
    vibrationsAmp[kEngineVibrationsChannel][Aileron] = static_cast<uint16_t>(aileronVibIntensity);
    vibrationsHz[kEngineVibrationsChannel][Aileron] = static_cast<uint16_t>(aileronVibCPS);
}