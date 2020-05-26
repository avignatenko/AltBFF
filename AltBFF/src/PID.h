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

    PIDController(double kp, double ki, double kd, double min, double max, double sampleTimeMs)
    {
        sampleTimeMs_ = sampleTimeMs;

        setTunings(kp, ki, kd);
        setOutputLimits(min, max);
    }


    void compute()
    {
        /*Compute all the working error variables*/
        const double error = setPoint_ - input_;
        iTerm_ += tiInv_ * error;
        iTerm_ = std::clamp(iTerm_, outMin_ - outputBase_, outMax_ - outputBase_);

        const double dInput = (input_ - lastInput.value());
        dInputAverage_.addSample(dInput);
 
        /*Compute PID output*/
        output_ = outputBase_ + kp_ * ( error + iTerm_ - td_ * dInputAverage_.get());
        output_ = std::clamp(output_, outMin_, outMax_);

        /*Remember some variables for next time*/
        lastInput = input_;
    }

    void setSimulatedOutput(double output)
    {
        output_ = outputBase_ + output;
        output_ = std::clamp(output_, outMin_, outMax_);
    }

    std::array<double, 9> dumpInternals()
    {
        return {kp_, tiInv_, td_, setPoint_,  input_, output_, setPoint_ - input_, iTerm_, -td_ * dInputAverage_.get() };
    }

    void setInput(double input) { input_ = input; if (!lastInput) lastInput = input; }
    double getInput() const { return input_; }

    void setSetPoint(double setPoint) { setPoint_ = setPoint; }

    void setOutputLimits(double min, double max)
    {
        if (min > max) return;
        outMin_ = min;
        outMax_ = max;

        output_ = std::clamp(output_, outMin_, outMax_);
        iTerm_ = std::clamp(iTerm_, outMin_ - outputBase_, outMax_ - outputBase_);
    }
    
    void setTunings(double kp, double ki, double kd)
    {
        double sampleTimeSec = sampleTimeMs_ / 1000.0;
        kp_ = kp;
        tiInv_ = sampleTimeSec / ki;
        td_ = kd / sampleTimeSec;
    }

    // 
    void setOutputBase(double base)
    {
        outputBase_ = base;
    }

    double getOutput() { return output_; }

private:

    double sampleTimeMs_ = 0.0;

    double input_ = 0.0;
    double outputBase_ = 0.0;
    double output_ = 0.0;
    double setPoint_ = 0.0;
    double iTerm_ = 0.0;
    std::optional<double> lastInput;
    double kp_ = 0.0;
    double tiInv_ = 0.0;
    double td_ = 0.0;
    double outMin_ = 0.0;
    double outMax_ = 0.0;

    MovingAverage<30> dInputAverage_;
};