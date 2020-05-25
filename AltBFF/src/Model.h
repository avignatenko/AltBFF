#pragma once

#include <Utils/Accumulators.h>

#include <spdlog/spdlog.h>
#include <cstdint>

// Model assumes it's running at certain rate, e.g. 30hz
class Model
{
public:
    struct Settings
    {
        struct Hardware
        {
            int aileronFrictionCoeff = 0;
            int aileronDumpingCoeff = 0;
            int aileronPositionFollowingP = 0;
            int aileronPositionFollowingI = 0;
            int aileronPositionFollowingD = 0;

            int elevatorFrictionCoeff = 0;
            int elevatorDumpingCoeff = 0;
            int elevatorPositionFollowingP = 0;
            int elevatorPositionFollowingI = 0;
            int elevatorPositionFollowingD = 0;
        };

        Hardware hardware;

        double clExponent = 0.0;

        double hTailPosLon = 0.0;
        double wingRootChord = 0.0;

        double elevatorArea = 0.0;
        double propWashElevatorCoeff = 0.0;
        double elevatorAlphaGain = 0.0;
        double elevatorAlphaScaleSpeedKn = 0.0;
        double elevatorPRGain = 0.0;
        double maxElevatorLift = 0.0;
        double maxElevatorAngleRadians = 0.0;
        double elevatorTrimGain = 0.0;

        double elevatorEngineFlowGain = 0.0;
        double elevatorEngineFreqGain = 0.0;
        double elevatorEngineFreqMin = 0.0;

        double elevatorVibStallGain = 0.0;
        double elevatorVibStalFreq = 0.0;

        double elevatorVibRunwayGain = 0.0;
        double elevatorVibRunwayFreq = 0.0;

        double aileronEngineFlowGain = 0.0;
        double aileronEngineFreqGain = 0.0;
        double aileronEngineFreqMin = 0.0;

        double aileronArea = 0.0;
        double propWashAileronCoeff = 0.0;
        double maxAileronLift = 0.0;
        double maxAileronAngleRadians = 0.0;
        double aileronTrimGain = 0.0;

        double aileronVibStallGain = 0.0;
        double aileronVibStalFreq = 0.0;

        double aileronVibRunwayGain = 0.0;
        double aileronVibRunwayFreq = 0.0;


       
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

    enum class GroundType
    {
        NA,
        Concrete,
        Grass,
        Noisy
    };

    void setGroundType(GroundType type)
    {
        groundType_ = type;
        spdlog::trace("Ground type set to model: {}", static_cast<int>(groundType_));
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
        alphaAngleRadAccum_.addSample(alpha);
        alphaAngleRad_ = alphaAngleRadAccum_.get();
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

    void setRelativeAoA(double relativeAoA)
    {
        relativeAoA_ = relativeAoA;
        spdlog::trace("relativeAoA set to model: {}", relativeAoA_);
    }

    // result

    int getFrictionCoeff(Axis axis);
    int getDumpingCoeff(Axis axis);
    
    int getPositionFollowingP(Axis axis);
    int getPositionFollowingI(Axis axis);
    int getPositionFollowingD(Axis axis);

    float getFixedForce(Axis axis) { return fixedForce_[axis]; }
    float getSpringForce(Axis axis) { return springForce_[axis]; }
    
    double getTotalForce(Axis axis);

    uint16_t getVibrationEngineHz(Axis axis) { return vibrationsHz_[0][axis]; }
    uint16_t getVibrationEngineAmp(Axis axis) { return vibrationsAmp_[0][axis]; }
    uint16_t getVibrationRunwayHz(Axis axis) { return vibrationsHz_[1][axis]; }
    uint16_t getVibrationRunwayAmp(Axis axis) { return vibrationsAmp_[1][axis]; }
    uint16_t getVibrationStallHz(Axis axis) { return vibrationsHz_[2][axis]; }
    uint16_t getVibrationStallAmp(Axis axis) { return vibrationsAmp_[2][axis]; }

    // update internal calculations
    void process();

    Settings settings_;
 private:

        void calculateElevatorForces();
        void calculateElevatorForces2();
        void calculateAileronForces();
        void calculateEngineVibrations();
        void calculateStallVibrations();
        void calculateRunwayVibrations();

        double calculateForceLiftDueToSpeed(double surfaceArea, double propWashCoeff);
private:

    // inputs
    double elevator_ = 0.0;
    double aileron_ = 0.0;
    double airDensity_ = 0.0;
    double tas_ = 0.0;
    double gs_ = 0.0;
    double thrust_ = 0.0;
    double alphaAngleRad_ = 0.0;
    double elevatorTrim_ = 0.0;
    double pitchRate_ = 0.0;
    double cgPosFrac_ = 0.0;
    double relativeAoA_ = 0.0; // [0, 100]
    int engine1RPM_ = 0;
    double engine1Flow_ = 0;
    bool onGround_ = false;
    GroundType groundType_ = GroundType::NA;

    MovingAverage<5> alphaAngleRadAccum_;

    // outputs
    MovingAverage<5> fixedForceAccum_[AxisCount];

    float fixedForce_[AxisCount] = {0.0f};
    float springForce_[AxisCount] = {0.0f};

    const int kEngineVibrationsChannel = 0;
    const int kRunwayVibrationChannel = 1;
    const int kStallVibrationsChannel = 2;


    uint16_t vibrationsHz_[3][AxisCount] = { 0 };
    uint16_t vibrationsAmp_[3][AxisCount] = { 0 };
};