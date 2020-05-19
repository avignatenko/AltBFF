#include <catch2/catch.hpp>
#include "Model.h"
#include "A2ASTec30AP.h"

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

TEST_CASE("AP Default")
{
    A2AStec30AP ap;

    REQUIRE(!ap.getCLAileron());
    REQUIRE(!ap.getCLElevator());

    REQUIRE(!ap.getSimAileron());
    REQUIRE(!ap.getSimElevator());

    ap.process();

    REQUIRE(!ap.getCLAileron());
    REQUIRE(!ap.getCLElevator());

    REQUIRE(!ap.getSimAileron());
    REQUIRE(!ap.getSimElevator());
}

TEST_CASE("AP roll")
{
    A2AStec30AP ap;

    ap.enableRollAxis(true);
    ap.setSimAileron(-50);
    ap.process();

    REQUIRE(ap.getCLAileron().value() == -50);
    REQUIRE(!ap.getCLElevator());

    REQUIRE(!ap.getSimAileron());
    REQUIRE(!ap.getSimElevator());

    ap.enableRollAxis(false);
    ap.process();

    REQUIRE(!ap.getSimAileron());
    REQUIRE(!ap.getSimElevator());
}

TEST_CASE("AP pitch")
{
    A2AStec30AP ap;

    ap.enablePitchAxis(true);
    ap.setSimElevator(-50);
    ap.process();

    REQUIRE(ap.getCLElevator().value() == -50 + 0.05);
    REQUIRE(!ap.getCLAileron());

    REQUIRE(!ap.getSimAileron());
    REQUIRE(ap.getSimElevator() == -50 + 0.05);

    ap.enablePitchAxis(false);
    ap.process();

    REQUIRE(!ap.getSimAileron());
    REQUIRE(!ap.getSimElevator());
}