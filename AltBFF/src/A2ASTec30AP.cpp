#include "A2ASTec30AP.h"

#include "Sim.h"
#include "Model.h"

#include <BFFCLAPI/CLStructures.h>

void A2AStec30AP::process(bffcl::CLInput& input)
{
    // pitch

    input.elevator.fixedForce = model_.getFixedForce(Model::Elevator);
    input.elevator.springForce = model_.getSpringForce(Model::Elevator);
    input.positionFollowingEngage &= ~(1u << 0); // clear pos following

    // roll
    if (sim_.readAxisControlState(Sim::Aileron) == Sim::AxisControl::Manual)
    {
        input.aileron.fixedForce = model_.getFixedForce(Model::Aileron);
        input.aileron.springForce = model_.getSpringForce(Model::Aileron);
        input.positionFollowingEngage &= ~(1u << 1); // clear pos following
    }
    else
    {
        input.aileron.fixedForce = 0;
        input.aileron.springForce = 0;
        input.positionFollowingEngage |= (1u << 1); // set pos following

        // fix for BFF CL acception [-100, -eps] instead of [-100, 100]
        const float bffMin = -100.0;
        const float bffMax = -0.2;
        float bffFixPos = std::clamp(((float)sim_.readAileron() + 100.0f) * (bffMax - bffMin) / 200.0f + bffMin, bffMin, bffMax);

        input.aileron.positionFollowingSetPoint = bffFixPos;

        spdlog::trace("Aileron in follow mode: {}", input.aileron.positionFollowingSetPoint);
    }
}