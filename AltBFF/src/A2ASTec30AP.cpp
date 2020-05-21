#include "A2ASTec30AP.h"

#include "Sim.h"
#include "Model.h"

#include <BFFCLAPI/CLStructures.h>


void A2AStec30AP::process()
{

    // aileron
    if (rollEnabled_)
    {
        aileronOut_ = simAileron_;
        spdlog::trace("AP aileron calculated: {}", aileronOut_);
    }

    // elevator
    if (pitchEnabled_)
    {
        double error = targetPressure_ - simPressure_;
        double errorDiff = 0.0; 
        if (prevElevatorError_) errorDiff = error - prevElevatorError_.value();

        prevElevatorError_ = error;

        double elevatorOffsetDueToPositionalError = settings_.pitchPID_.p * error;
        double elevatorOffsetDueToErrorDerivative = settings_.pitchPID_.d * errorDiff;
        double elevatorOffset =  (elevatorOffsetDueToPositionalError + elevatorOffsetDueToErrorDerivative);


        // note: we're using sim/cl elevator here (fixme)
        elevatorOut_ = std::clamp(elevatorOut_ + elevatorOffset, -100.0, 100.0); // fixme: clamping makes wrong results with I != 0

        spdlog::trace("error: {}, errorDiff: {}, offsetP: {}, offsetD: {}", error, errorDiff, elevatorOffsetDueToPositionalError, elevatorOffsetDueToErrorDerivative);
        spdlog::trace("total offset elevator: {}", elevatorOffset);
        spdlog::debug("AP elevator calculated: {}", elevatorOut_);    
    }
}

std::optional<double> A2AStec30AP::getCLAileron()
{
    return rollEnabled_ ? aileronOut_ : std::optional<double>();
}

std::optional<double> A2AStec30AP::getCLElevator()
{
    return pitchEnabled_ ? elevatorOut_ : std::optional<double>();
}

std::optional<double> A2AStec30AP::getSimAileron()
{
    return std::optional<double>(); // we follow and not write ailerons
}

std::optional<double> A2AStec30AP::getSimElevator()
{
    return getCLElevator();
}