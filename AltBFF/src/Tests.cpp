#include <catch2/catch.hpp>
#include "Model.h"

namespace
{
static double kPi = std::acos(-1);
}

class StandardSettingsTestFixture {
protected:
    Model::Settings s_;
public:
    StandardSettingsTestFixture()
    {
        s_.clExponent = 2.0;
        s_.elevatorArea = 2.0;
        s_.maxElevatorLift = 2.83;
        s_.maxElevatorAngleRadians = 17.0 / 180 * kPi;
    }

};

TEST_CASE_METHOD(StandardSettingsTestFixture, "test elevator forces balance")
{
    s_.propWashElevatorCoeff = 0.5;
    s_.elevatorAlphaGain = -1.0;
    s_.elevatorTrimGain = 1.0;

    s_.elevatorAlphaScaleSpeedKn = 0.0;
    s_.elevatorPRGain = 0.0;

    Model model(s_);

    // set model params
    model.setElevator(1.0);
    model.setElevatorTrim(1.0);
    model.setAirDensity(1.2);
    model.setAlpha(0.0);
    model.setOnGround(false);
    model.setTAS(40);

    model.process();

    double springForce = model.getSpringForce(Model::Elevator);
    double fixedForce = model.getFixedForce(Model::Elevator);
    REQUIRE(springForce == 0.54335999488830566);
    REQUIRE(fixedForce == 54.335998535156250);

}