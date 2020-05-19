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
        elevatorOut_ = targetPressure_; // just for test
        spdlog::trace("AP elevator calculated: {}", elevatorOut_);    
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