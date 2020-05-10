#pragma once

#include <spdlog/spdlog.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

#include <cstdint>

// Model assumes it's running at certain rate, e.g. 30hz
class Model
{
public:
    struct Settings
    {
        int aileronFrictionCoeff = 0;
        int aileronDumpingCoeff = 0;
        int elevatorFrictionCoeff = 0;
        int elevatorDumpingCoeff = 0;

        double clExponent = 0.0;

        double hTailPosLon = 0.0;
        double wingRootChord = 0.0;

        double elevatorArea = 0.0;
        double propWashElevatorCoeff = 0.0;
        double elevatorAlphaGain = 0.0;
        double elevatorPRGain = 0.0;
        double maxElevatorLift = 0.0;
        double maxElevatorAngleRadians = 0.0;
        double elevatorTrimGain = 0.0;

        double elevatorEngineFlowGain = 0.0;
        double elevatorEngineFreqGain = 0.0;
        double elevatorEngineFreqMin = 0.0;

        double aileronEngineFlowGain = 0.0;
        double aileronEngineFreqGain = 0.0;
        double aileronEngineFreqMin = 0.0;

        double aileronArea = 0.0;
        double propWashAileronCoeff = 0.0;
        double maxAileronLift = 0.0;
        double maxAileronAngleRadians = 0.0;
        double aileronTrimGain = 0.0;

       
    };

    Model(const Settings& settings);

    void setSettings(const Settings& settings);

    enum Axis
    {
        Elevator = 0,
        Aileron,
        AxisCount,
    };

    // inputs

    // +/- 100%, mid = 0
    void setElevator(double elevator) 
    { 
        elevator_ = elevator; 
        spdlog::trace("Elevator set to model: {}", elevator_);
    }

    // +/- 100%, mid = 0
    void setAileron(double aileron) 
    { 
        aileron_ = aileron; 
        spdlog::trace("Aileron set to model: {}", aileron_);
    }

    // kg / m^3
    void setAirDensity(double density)
    {
        airDensity_ = density;
        spdlog::trace("Air density set to model: {}", airDensity_);
    }

    // tas in ms/sec
    void setTAS(double tas) 
    { 
        tas_ = tas; 
        spdlog::trace("TAS set to model: {}", tas_);
    }

    void setGS(double gs)
    {
        gs_ = gs;
        spdlog::trace("GS set to model: {}", gs_);
    }

    // true if on ground
    void setOnGround(bool onGround)
    {
        onGround_ = onGround;
        spdlog::trace("onGround set to model: {}", onGround_);
    }

    // thrust in pounds
    void setThrust(double thrust)
    { 
        thrust_ = thrust; 
        spdlog::trace("Thrust set to model: {}", thrust_);
    }


    // incidence "alpha" in radians
    void setAlpha(double alpha)
    {
        alphaAngleRadAccum_(alpha);
        alphaAngleRad_ = boost::accumulators::rolling_mean(alphaAngleRadAccum_);
        spdlog::trace("Alpha set to model: {}, average: {}", alpha, alphaAngleRad_);
    }

    // Pitch velocity in rads/sec relative to the body axes
    void setPitchRate(double pitchRate)
    {
        pitchRate_ = pitchRate;
        spdlog::trace("Pitch rate set to model: {}", pitchRate_);
    }

    void setCGPosFrac(double ccPosFrac)
    {
        cgPosFrac_ = ccPosFrac;
        spdlog::trace("GC Pos Frac rate set to model: {}", cgPosFrac_);
    }

    // [-1, 1]
    void setElevatorTrim(double clElevatorTrim) { 
        elevatorTrim_ = clElevatorTrim; 
        spdlog::trace("Elevator trim set to model: {}", elevatorTrim_);
    }
    
    // rot per minute
    void setEngine1RPM(int rpm)
    {
        engine1RPM_ = rpm;
        spdlog::trace("Engine1 rpm set to model: {}", engine1RPM_);
    }

    void setEngine1Flow(double flow)
    {
        engine1Flow_ = flow;
        spdlog::trace("Engine1 flow set to model: {}", engine1Flow_);
    }

    // result

    int getFrictionCoeff(Axis axis);
    int getDumpingCoeff(Axis axis);

    float getFixedForce(Axis axis) { return fixedForce_[axis]; }
    float getSpringForce(Axis axis) { return springForce_[axis]; }
    uint16_t getVibrationCh1Hz(Axis axis) { return vibrationsHz[0][axis]; }
    uint16_t getVibrationCh1Amp(Axis axis) { return vibrationsAmp[0][axis]; }
    uint16_t getVibrationCh2Hz(Axis axis) { return vibrationsHz[1][axis]; }
    uint16_t getVibrationCh2Amp(Axis axis) { return vibrationsAmp[1][axis]; }
    uint16_t getVibrationCh3Hz(Axis axis) { return vibrationsHz[2][axis]; }
    uint16_t getVibrationCh3Amp(Axis axis) { return vibrationsAmp[2][axis]; }

    // update internal calculations
    void process();

 private:

        void calculateElevatorForces();
        void calculateAileronForces();
        void calculateEngineVibrations();

        double calculateForceLiftDueToSpeed(double surfaceArea, double propWashCoeff);
private:

    using Accumulator = boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::rolling_mean> >;
    using AccParams = boost::accumulators::tag::rolling_window;
   
    Settings settings_;

    // inputs
    double elevator_ = 0.0;
    double aileron_ = 0.0;

    double airDensity_ = 0.0;
    double tas_ = 0.0;
    double gs_ = 0.0;
    double thrust_ = 0.0;
    double alphaAngleRad_ = 0.0;
    Accumulator alphaAngleRadAccum_ = Accumulator(AccParams::window_size = 5);
    double elevatorTrim_ = 0.0;
    double pitchRate_ = 0.0;
    double cgPosFrac_ = 0.0;

    int engine1RPM_ = 0;
    double engine1Flow_ = 0;

    bool onGround_ = false;

    // outputs
    Accumulator fixedForceAccum_[AxisCount] = 
    { 
        Accumulator(AccParams::window_size = 5), 
        Accumulator(AccParams::window_size = 5) 
    };

    float fixedForce_[AxisCount] = {0.0f};
    float springForce_[AxisCount] = {0.0f};

    uint16_t vibrationsHz[3][AxisCount] = { 0 };
    uint16_t vibrationsAmp[3][AxisCount] = { 0 };
};