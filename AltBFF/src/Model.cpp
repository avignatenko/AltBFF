#include "Model.h"

#include <cmath>

#include <spdlog/spdlog.h>

Model::Model(const Settings& settings) : settings_(settings) {}

namespace
{
double kAirDensity = 1.2;
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
}

void Model::calculateElevatorForces()
{
    double elevatorDeflectionAngleRad = (elevator_ / 100.0) * settings_.maxElevatorAngleRadians;
    double clCoeffElevator = settings_.maxElevatorLift / settings_.maxElevatorAngleRadians;
    double clElevator = clCoeffElevator * elevatorDeflectionAngleRad;

    // add prop wash airspeed to normal airspeed (simple model)
    double propWashAirSpeed = std::pow(thrust_ / 10.0, 0.8);
    double airSpeed = tas_ + (propWashAirSpeed * settings_.propWashCoeff);

    // Calculate lift force for elevator
    // From lift equation: L = Cl * pho * V^2 * A / 2 ((https://www.grc.nasa.gov/www/k-12/airplane/lifteq.html)
    double flElevatorDueToSpeed =
        settings_.elevatorArea / 100.0 * kAirDensity / 2.0 * std::pow(airSpeed, settings_.clExponent);

    double flElevator = clElevator * flElevatorDueToSpeed;

    // spring force measures from max elevator force
    double clElevatorMax = clCoeffElevator * settings_.maxElevatorAngleRadians;
    double flElevatorSpring = clElevatorMax * flElevatorDueToSpeed / 255;

    // trim works same as elevator, but with gain (effectiveness)
    double elevatorTrimDeflectionAngleRad = elevatorTrim_ * settings_.maxElevatorAngleRadians;
    double clElevatorTrim = clCoeffElevator * elevatorTrimDeflectionAngleRad;

    double fElevatorTrim = clElevatorTrim * flElevatorDueToSpeed * settings_.elevatorTrimGain;

    double fStickPusherElevator = 0.0;
    double fAlphaElevator = 0.0;
    double fElevatorWeight = 0.0;

    double flElevatorFixed = fStickPusherElevator + fAlphaElevator + fElevatorTrim + fElevatorWeight;

    spdlog::debug(
        "Model vars: elevatorDeflectionAngleRad: {}, clCoeffElevator: {}, clElevator: {}, propWashAirSpeed: {}, "
        "airSpeed: {}, "
        "tas: {}, flElevator: {}, flElevatorSpring: {}",
        elevatorDeflectionAngleRad, clCoeffElevator, clElevator, propWashAirSpeed, airSpeed, tas_, flElevator,
        flElevatorSpring);

    spdlog::debug("Elevator trim: {}", elevatorTrim_);

    spdlog::debug("Spring force: {}, Fixed Force: {}", flElevatorSpring, flElevatorFixed);
    // test update elevator
    fixedForce_[Elevator] = flElevatorFixed;
    springForce_[Elevator] = flElevatorSpring;
}
