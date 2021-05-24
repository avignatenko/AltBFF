#pragma once

#include <Utils/Accumulators.h>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <optional>

// after http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
// controller is calculated in velocity form after
// https://www.philadelphia.edu.jo/academics/kaubaidy/uploads/RTS-Lec6N.pdf
class PIDController
{
public:
    PIDController(double kp, double ti, double td, double duMin, double duMax, double min, double max,
                  double sampleTimeMs, double input, double output)
    {
        sampleTimeMs_ = sampleTimeMs;
        kp_ = kp;
        ti_ = ti;
        td_ = td;
        output_ = lastOutput_ = output0_ = output;

        setOutputLimits(duMin, duMax, min, max);

        updateCoeffs();
    }

    void compute()
    {
        // update last variables before making new
        error_[2] = error_[1];
        error_[1] = error_[0];
        error_[0] = setPoint_ - input_;

        lastOutput_ = output_;

        double du = std::clamp(k1_ * error_[0] + k2_ * error_[1] + k3_ * error_[2], outDuMin_, outDuMax_);
        output_ = std::clamp(lastOutput_ + du, outMin_, outMax_);
    }

    void setManualOutput(double output)
    {
        output_ = output0_ + output;
        output_ = std::clamp(output_, outMin_, outMax_);

        lastOutput_ = output_;
    }

    std::array<double, 9> dumpInternals()
    {
        return {k1_, k2_, k3_, setPoint_, error_[0], error_[1], error_[2], input_, output_};
    }

    void setInput(double input) { input_ = input; }
    double getInput() const { return input_; }

    void setSetPoint(double setPoint) { setPoint_ = setPoint; }

    void setOutputLimits(double duMin, double duMax, double min, double max)
    {
        outMin_ = min;
        outMax_ = max;
        outDuMin_ = duMin;
        outDuMax_ = duMax;

        output_ = std::clamp(output_, outMin_, outMax_);
    }

    void setTunings(double kp, double ti, double td)
    {
        kp_ = kp;
        ti_ = ti;
        td_ = td;
        updateCoeffs();
    }

    void setSampleTimeMs(double timeMs)
    {
        sampleTimeMs_ = timeMs;
        updateCoeffs();
    }

    double getOutput() { return output_; }

private:
    double getSampleTimeSec() const { return sampleTimeMs_ / 1000.0; }

    void updateCoeffs()
    {
        double ts = getSampleTimeSec();
        k1_ = kp_ * (1 + ts / ti_ + td_ / ts);
        k2_ = -kp_ * (1 + 2 * td_ / ts);
        k3_ = kp_ * td_ / ts;
    }

private:
    double sampleTimeMs_ = 0.0;

    double input_ = 0.0;
    double setPoint_ = 0.0;

    double output_ = 0.0;
    double output0_ = 0.0;
    double lastOutput_ = 0.0;

    double outMin_ = 0.0;
    double outMax_ = 0.0;

    double outDuMin_ = 0.0;
    double outDuMax_ = 0.0;

    double kp_ = 0.0;
    double ti_ = 0.0;
    double td_ = 0.0;

    double k1_ = 0.0;
    double k2_ = 0.0;
    double k3_ = 0.0;

    double error_[3] = {0.0};
};