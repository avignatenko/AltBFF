#include "Model.h"

#include <cmath>

#include <spdlog/spdlog.h>

Model::Model(const Settings& settings) : settings_(settings) {}

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
    double kAirDensity = 1.2;

    double elevatorDeflectionAngleRad = (elevator_ / 100.0) * settings_.maxElevatorAngleRadians;
    double clCoeffElevator = settings_.maxElevatorLift / settings_.maxElevatorAngleRadians;
    double clElevator = clCoeffElevator * elevatorDeflectionAngleRad;

    // add prop wash airspeed to normal airspeed
    double propWash = std::pow(thrust_ / 10.0, 0.8);
    double airSpeed = tas_ + (propWash * settings_.propWashCoeff);

    // Calculate lift force for elevator
    double flElevator =
        clElevator * settings_.elevatorArea / 100.0 * kAirDensity / 2.0 * std::pow(airSpeed, settings_.clExponent);

    double flElevatorSpring = clCoeffElevator * settings_.maxElevatorAngleRadians * settings_.elevatorArea *
                              kAirDensity / 2.0 * std::pow(airSpeed, settings_.clExponent) / 10.0 / 2550;

    spdlog::info(
        "Model vars: elevatorDeflectionAngleRad: {}, clCoeffElevator: {}, clElevator: {}, propWash: {}, airSpeed: {}, "
        "tas: {}, flElevator: {}, flElevatorSpring: {}",
        elevatorDeflectionAngleRad, clCoeffElevator, clElevator, propWash, airSpeed, tas_, flElevator,
        flElevatorSpring);

    // test update elevator
    // fixedForce_[Elevator] = -flElevator;
    springForce_[Elevator] = flElevatorSpring;
}
