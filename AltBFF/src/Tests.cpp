#include "Model.h"
#include "A2ASTec30AP.h"
#include "PID.h"

#include <Utils/Accumulators.h>
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
    s.elevatorPID = { 50, 0, 50 };
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
    PIDController p(-1.0/10, 10, 0, 200.0 / (1.0 / 30), -100, 100, 1000.0 / 30, 10100, -100);

    auto syst = [](double output) { return 10000 - (output + 30); };

    p.setSetPoint(10000);
    p.setInput(syst(-30));


    double output = 0;
    for (int i = 0; i < 10000; ++i)
    {
        p.compute();
        output = p.getOutput();
        p.setInput(syst(output));
    }


 
     output = p.getOutput();
//    REQUIRE(output == -30.0);

    p.setInput(10050);
    p.compute();

    output = p.getOutput();
  //  REQUIRE(output == -15.299999999999999);

}

TEST_CASE("AP pitch pitch 1")
{
    A2AStec30AP::Settings s;
    s.elevatorPID = { 50, 10, 0 };
    A2AStec30AP ap(s);

    ap.setSimElevator(-50);
    ap.setSimPitch(0);
    ap.enablePitchAxis(true);
    ap.process();

    ap.setSimPitch(-10.0 / 180.0 * 3.14); // pitch up
    ap.process();
    REQUIRE(ap.getSimElevator() == -41.220211111111112);
    ap.process();
    REQUIRE(ap.getSimElevator() == -32.382855555555558);
    ap.process();
    REQUIRE(ap.getSimElevator() == -23.487933333333338);
}

TEST_CASE("AP pitch pitch 2")
{
    A2AStec30AP::Settings s;
    s.elevatorPID = { 50, 400, 0 };
    A2AStec30AP ap(s);

    ap.setSimElevator(-50);
    ap.setSimPitch(0);
    ap.enablePitchAxis(true);
    ap.process();

    ap.setSimPitch(-40.0 / 180.0 * 3.14); // pitch up
    ap.process();
    REQUIRE(ap.getSimElevator() == -5.9004444444444459);
    ap.process();
    REQUIRE(ap.getSimElevator() == 47.409777777777776);
    ap.process();
    REQUIRE(ap.getSimElevator() == 100.0);
}

TEST_CASE("AP pitch pitch 3")
{
    A2AStec30AP::Settings s;
    s.elevatorPID = { 50, 400, 0 };
    A2AStec30AP ap(s);

    ap.setSimElevator(-50);
    ap.setSimPitch(0);
    ap.enablePitchAxis(true);
    ap.process();

    ap.setSimPitch(-40.0 / 180.0 * 3.14); // pitch up
    ap.process();
    REQUIRE(ap.getSimElevator() == -5.9004444444444459);
    ap.process();
    REQUIRE(ap.getSimElevator() == 47.409777777777776);
    ap.process();
    REQUIRE(ap.getSimElevator() == 100.0);
}

TEST_CASE("Moving average")
{
    MovingAverage<5> ma;

    REQUIRE(ma.get() == 0.0);

    ma.addSample(1.5);

    REQUIRE(ma.get() == 1.5);

    ma.addSample(3);

    REQUIRE(ma.get() == 2.25);

    ma.addSample(3);
    ma.addSample(3);
    ma.addSample(3);

    REQUIRE(ma.get() == 2.7);

    ma.addSample(3);

    REQUIRE(ma.get() == 3);

    ma.addSample(0);

    REQUIRE(ma.get() == 2.4);

    for (int i = 0; i < 5; ++i)
        ma.addSample(1);

    REQUIRE(ma.get() == 1.0);

    // test for cached
    REQUIRE(ma.get() == 1.0);

}

TEST_CASE("AP Impulse Response")
{
    A2AStec30AP::Settings s;
    s.pitchmode = -1;
    s.doStepResponse = true;
    s.stepResponseInputFile = "elev_sr.csv";
    A2AStec30AP ap(s);

    ap.enablePitchAxis(true);

    for (int i = 0; i < 30 * 20; ++i)
    {
        ap.setSimPitchRate(i);
        ap.process();
    }
}