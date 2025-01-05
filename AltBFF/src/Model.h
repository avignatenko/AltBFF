#pragma once

#include <Utils/Accumulators.h>
#include <Utils/Common.h>
#include <spdlog/spdlog.h>
#include <array>
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
        double elevatorNeutralPos = 0.0;
        double propWashElevatorCoeff = 0.0;
        bool calculatePropWash = true;
        double elevatorAlphaGain = 0.0;
        double elevatorAlphaScaleSpeedKn = 0.0;
        double elevatorPRGain = 0.0;
        double maxElevatorLift = 0.0;
        double maxElevatorAngleRadians = 0.0;
        double elevatorTrimGain = 0.0;
        double elevatorTrimNeutralPos = 0.0;

        double engineVibAirGain = 0.0;
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

        bool forceTrimIntoSim = false;
        double forcedSimTrim = 0.0;
    };

    explicit Model(const Settings& settings);

    void setSettings(const Settings& settings);

    enum Axis
    {
        Elevator = 0,
        Aileron
    };

    constexpr static int AxisCount = 2;

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
        if (airDensity_ < 1) airDensity_ = kBaseAirDensity;
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
    void setElevatorTrim(double clElevatorTrim)
    {
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

    // in m/s
    void setPropWash(double propWash)
    {
        propWash_ = propWash;
        spdlog::trace("propWash set to model: {}", propWash_);
    }

    // result

    [[nodiscard]] int getFrictionCoeff(Axis axis) const;
    [[nodiscard]] int getDumpingCoeff(Axis axis) const;

    [[nodiscard]] int getPositionFollowingP(Axis axis) const;
    [[nodiscard]] int getPositionFollowingI(Axis axis) const;
    [[nodiscard]] int getPositionFollowingD(Axis axis) const;

    [[nodiscard]] float getFixedForce(Axis axis) const { return fixedForce_[axis]; }
    [[nodiscard]] float getSpringForce(Axis axis) const { return springForce_[axis]; }

    [[nodiscard]] double getTotalForce(Axis axis) const;

    [[nodiscard]] uint16_t getVibrationEngineHz(Axis axis) const { return vibrationsEngine_[axis].hz; }
    [[nodiscard]] uint16_t getVibrationEngineAmp(Axis axis) const { return vibrationsEngine_[axis].amp; }
    [[nodiscard]] uint16_t getVibrationRunwayHz(Axis axis) const { return vibrationsRunway_[axis].hz; }
    [[nodiscard]] uint16_t getVibrationRunwayAmp(Axis axis) const { return vibrationsRunway_[axis].amp; }
    [[nodiscard]] uint16_t getVibrationStallHz(Axis axis) const { return vibrationsStall_[axis].hz; }
    [[nodiscard]] uint16_t getVibrationStallAmp(Axis axis) const { return vibrationsStall_[axis].amp; }

    [[nodiscard]] bool getForceTrimIntoSim() const { return settings_.forceTrimIntoSim; }
    [[nodiscard]] double getForcedSimElevatorTrim() const { return settings_.forcedSimTrim; }

    // update internal calculations
    void process();

    Settings settings_;

private:
    void calculateElevatorForces();
    // void calculateElevatorForces2();
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
    double propWash_ = 0.0;
    double alphaAngleRad_ = 0.0;
    double elevatorTrim_ = 0.0;
    double pitchRate_ = 0.0;
    double cgPosFrac_ = 0.0;
    double relativeAoA_ = 0.0;  // [0, 100]
    int engine1RPM_ = 0;
    double engine1Flow_ = 0;
    bool onGround_ = false;
    GroundType groundType_ = GroundType::NA;

    MovingAverage<5> alphaAngleRadAccum_;

    // outputs
    std::array<MovingAverage<5>, AxisCount> fixedForceAccum_;

    std::array<float, AxisCount> fixedForce_ = {0.0f};
    std::array<float, AxisCount> springForce_ = {0.0f};

    struct Vibrations
    {
        uint16_t hz = 0;
        uint16_t amp = 0;
    };

    std::array<Vibrations, AxisCount> vibrationsEngine_;
    std::array<Vibrations, AxisCount> vibrationsRunway_;
    std::array<Vibrations, AxisCount> vibrationsStall_;
};
