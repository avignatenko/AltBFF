#include "Model.h"

#include <cmath>

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

    // Calculate lift force for elevator
    double flElevator =
        clElevator * (settings_.elevatorArea / 100.0) * kAirDensity / 2.0 * std::pow(tas_, settings_.clExponent);

    // test update elevator
    fixedForce_[Elevator] = -flElevator;
}
