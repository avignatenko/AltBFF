#pragma once

#include <Utils/Accumulators.h>

#include <spdlog/spdlog.h>

#include <optional>
#include <algorithm>
#include <array>


// after http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
class PIDController
{
public:

    PIDController(double kp, double ti, double td, double  duMin, double duMax, double min, double max, double sampleTimeMs, double input, double output)
    {
        sampleTimeMs_ = sampleTimeMs;

        output_ = lastOutput_ = output0_ = output;

        setTunings(kp, ti, td);
        setOutputLimits(duMin, duMax, min, max);
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
        return {k1_, k2_, k3_, setPoint_,  error_[0], error_[1], error_[2], input_, output_ };
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
        double ts = getSampleTimeSec();
        k1_ = kp * (1 + ts / ti + td / ts);
        k2_ = -kp * (1 + 2 * td / ts);
        k3_ = kp * td / ts;
    }

    double getOutput() { return output_; }

private:

    double getSampleTimeSec() const { return sampleTimeMs_ / 1000.0;  }

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

    double k1_ = 0.0;
    double k2_ = 0.0;
    double k3_ = 0.0;

    double error_[3] = { 0.0 };
};
