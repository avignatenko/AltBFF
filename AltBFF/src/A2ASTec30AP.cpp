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
        PIDController& pitchController = pitchController_.value();
        pitchController.setInput(simPitch_);
        pitchController.setOutputLimits(-elevatorOut_ - 100.0, 100.0 - elevatorOut_);
        pitchController.compute();

        // note: we're using sim/cl elevator here (fixme)
        elevatorOut_ = std::clamp(elevatorOut_ + pitchController.getOutput(), -100.0, 100.0); // fixme: clamping makes wrong results with I != 0

        auto internals = pitchController.dumpInternals();
        spdlog::trace("termP: {}, termI: {}, termD: {}", std::get<0>(internals), std::get<1>(internals), std::get<2>(internals) );
        spdlog::trace("total offset elevator: {}", pitchController.getOutput());
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