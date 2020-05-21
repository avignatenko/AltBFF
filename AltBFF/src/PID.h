#pragma once

#include <optional>
#include <algorithm>
#include <tuple>

// after http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
class PIDController
{
public:

    PIDController(double kp, double ki, double kd, double min, double max, long sampleTimeMs)
    {
        double sampleTimeInSec = sampleTimeMs / 1000.0;
        kp_ = kp;
        ki_ = ki * sampleTimeInSec;
        kd_ = kd / sampleTimeInSec;
        setOutputLimits(min, max);
    }

    void compute()
    {
        /*Compute all the working error variables*/
        double error = setPoint_ - input_;
        iTerm_ += (ki_ * error);
        iTerm_ = std::clamp(iTerm_, outMin_, outMax_);

        double dInput = (input_ - lastInput.value());

        /*Compute PID output*/
        output_ = kp_ * error + iTerm_ - kd_ * dInput;
        output_ = std::clamp(output_, outMin_, outMax_);

        /*Remember some variables for next time*/
        lastInput = input_;
    }

    std::tuple<double, double, double> dumpInternals()
    {
        return { kp_ * (setPoint_ - input_), iTerm_, - kd_ * (input_ - lastInput.value()) };
    }

    void setInput(double input) { input_ = input; if (!lastInput) lastInput = input; }
    void setSetPoint(double setPoint) { setPoint_ = setPoint; }

    void setOutputLimits(double min, double max)
    {
        if (min > max) return;
        outMin_ = min;
        outMax_ = max;

        output_ = std::clamp(output_, outMin_, outMax_);
        iTerm_ = std::clamp(iTerm_, outMin_, outMax_);
    }

    double getOutput() { return output_; }

private:

    double input_ = 0.0;
    double output_ = 0.0;
    double setPoint_ = 0.0;
    double iTerm_ = 0.0;
    std::optional<double> lastInput;
    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;
    double outMin_ = 0.0;
    double outMax_ = 0.0;
};