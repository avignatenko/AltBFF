#include "Model.h"
#include "A2ASTec30AP.h"
#include "PID.h"
#include <catch2/catch.hpp>
#include <iostream>

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
    model.setElevator(100.0);
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

    double totalForce = model.getTotalForce(Model::Elevator);
    REQUIRE(std::abs(totalForce) < 1e-5);
}

TEST_CASE("AP Default")
{
    A2AStec30AP::Settings s;
    A2AStec30AP ap(s);

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
    A2AStec30AP::Settings s;
    A2AStec30AP ap(s);

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
    A2AStec30AP::Settings s;
    A2AStec30AP ap(s);

    ap.enablePitchAxis(true);
    ap.setSimElevator(-50);
    ap.process();

    REQUIRE(ap.getCLElevator().value() == -50);
    REQUIRE(!ap.getCLAileron());

    REQUIRE(!ap.getSimAileron());
    REQUIRE(ap.getSimElevator() == -50);

    ap.enablePitchAxis(false);
    ap.process();

    REQUIRE(!ap.getSimAileron());
    REQUIRE(!ap.getSimElevator());
}

TEST_CASE("AP pitch 2")
{
    A2AStec30AP::Settings s;
    s.pitchPID_ = { 50, 0, 50 };
    A2AStec30AP ap(s);

    ap.setSimElevator(-50);
    ap.setAirPressure(84307.28); // at 5000 feet
    ap.enablePitchAxis(true);
    ap.process();

    ap.setAirPressure(84149.64); // at 5050 feet
    ap.process();
    //std::cout << ap.getCLElevator();
   
}

TEST_CASE("PID")
{
    PIDController p(0.3, 5, 1, 1000 / 30);
    p.SetOutputLimits(-100, 100);

    p.setSetPoint(10000);
    p.setInput(10100);

    p.compute();

    double output = p.getOutput();
    REQUIRE(output == -46.5);
}