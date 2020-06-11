#include "Model.h"
#include "A2ASTec30AP.h"
#include <Utils/PID.h>
#include <Utils/RateLimiter.h>

#include <Utils/Accumulators.h>
#include <Utils/Common.h>
#include <catch2/catch.hpp>
#include <iostream>

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

    ap.setSimElevator(-50);
    ap.enablePitchAxis(true);
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

TEST_CASE("AP pitch: maintain pitch")
{
    A2AStec30AP::Settings s;
    s.elevatorPID = { 100, 10000, 0 };
    s.pitchmode = 0;
    A2AStec30AP ap(s);

    ap.setSimElevator(-50);
    ap.setSimPitch(0);
    ap.enablePitchAxis(true);
    ap.process();
    REQUIRE(ap.getSimElevator() == -50);

    ap.setSimPitch(degToRad(-5));
    ap.process();
    REQUIRE(ap.getSimElevator() == -41.273324651207489);
   
    //std::cout << ap.getCLElevator();

}


TEST_CASE("AP pitch pitch 1")
{
    A2AStec30AP::Settings s;
    s.elevatorPID = { 50, 1, 0 };
    A2AStec30AP ap(s);

    ap.setSimElevator(-50);
    ap.setSimPitch(0);
    ap.enablePitchAxis(true);
    ap.process();

    ap.setSimPitch(degToRad(-10.0)); // pitch up
    ap.process();
    REQUIRE(ap.getSimElevator() == -40.982465531362635);
    ap.process();
    REQUIRE(ap.getSimElevator() == -40.691577322696915);
    ap.process();
    REQUIRE(ap.getSimElevator() == -40.400689114031188);
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

TEST_CASE("Rate limiter")
{
    RateLimiter r(-10, 20, -100, 1000.0/30.0);

    SECTION("check low and high steps")
    {
        r.setInput(-100);
        r.process();
        REQUIRE(r.getOutput() == -100.0);

        r.setInput(-99);
        r.process();
        REQUIRE(r.getOutput() == -99.333333333333329);

        r.setInput(0);
        r.process();
        REQUIRE(r.getOutput() == -98.666666666666657);

        r.setInput(-98.166666666666657);
        r.process();
        REQUIRE(r.getOutput() == -98.166666666666657);

        r.setInput(-98.166666666666657);
        r.process();
        REQUIRE(r.getOutput() == -98.166666666666657);
    }

    // make a cycle
    SECTION("check integral time as expected")
    {
        for (int i = 0; i < 20 * 30; ++i)
        {
            r.setInput(100);
            r.process();
        }

        REQUIRE(r.getOutput() == 100.0);

        for (int i = 0; i < 20 * 30; ++i)
        {
            r.setInput(-100);
            r.process();
        }

        REQUIRE(r.getOutput() == 0.0);
    }
 
}


TEST_CASE("Test EMA")
{
    ExponentialMovingAverage a(1);
    a.addSample(10);
    REQUIRE(a.get() == 10);
    a.addSample(15);
    REQUIRE(a.get() == 15);

    ExponentialMovingAverage b(3);
    b.addSample(10);
    REQUIRE(b.get() == 10);
    b.addSample(15);
    REQUIRE(b.get() == 12.5);
}