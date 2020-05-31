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

    PIDController(double kp, double ti, double td, double duMax, double min, double max, double sampleTimeMs, double input, double output)
    {
        sampleTimeMs_ = sampleTimeMs;

        lastInput_ = input;
        lastOutput_ = output; 
        spdlog::trace("Initial iTerm: {}, as {} / {}", iTerm_, output, kp);
        setTunings(kp, ti, td);
        setOutputLimits(duMax, min, max);
    }


    void compute()
    {
        // Compute all the working error variables
        const double error = setPoint_ - input_;

        spdlog::trace("Compute iTerm: before {}", iTerm_);
        iTerm_ += ki_ * error;
        spdlog::trace("Compute iTerm: add error {}", iTerm_);
        iTerm_ = std::clamp(iTerm_, outMin_ - lastOutput_, outMax_ - lastOutput_);
        spdlog::trace("Compute iTerm: clip {}, with {} - {}", iTerm_, outMin_, outMax_);

        const double dInput = input_ - lastInput_;
        //dInputAverage_.addSample(dInput);
 
        /// Compute PID output
        
        double output =  kp_ * error + iTerm_ - kd_ * dInput; // dInputAverage_.get());
        
        // clamp output
        double dOutput = std::clamp(output, -outDuMax_, outDuMax_);
        output_ = std::clamp(lastOutput_ + dOutput, outMin_, outMax_);
        
        // Remember some variables for next time
        lastInput_ = input_;
        lastOutput_ = output_;
    }

    void setSimulatedOutput(double output)
    {
        output_ = lastOutput_ + output;
        output_ = std::clamp(output_, outMin_, outMax_);

        lastOutput_ = output_;
    }


    std::array<double, 9> dumpInternals()
    {
        return {kp_, ki_, kd_, setPoint_,  input_, output_, setPoint_ - input_, iTerm_, -kd_ * (input_ - lastInput_)  };
    }

    void setInput(double input) { input_ = input; }
    double getInput() const { return input_; }

    void setSetPoint(double setPoint) { setPoint_ = setPoint; }

    void setOutputLimits(double duMax, double min, double max)
    {
        const double sampleTimeSec = sampleTimeMs_ / 1000.0;

        outMin_ = min;
        outMax_ = max;
        outDuMax_ = duMax * sampleTimeSec;

        output_ = std::clamp(output_, outMin_, outMax_);
        iTerm_ = std::clamp(iTerm_, outMin_- lastOutput_, outMax_ - lastOutput_);
    }
    
    void setTunings(double kp, double ti, double td)
    {
        const double sampleTimeSec = sampleTimeMs_ / 1000.0;
        kp_ = kp;
        ki_ = (kp / ti) * sampleTimeSec;
        kd_ = (kp * td) / sampleTimeSec;
    }

    double getOutput() { return output_; }

private:

    double sampleTimeMs_ = 0.0;

    double input_ = 0.0;
    double lastInput_ = 0.0;
    double output_ = 0.0;
    double lastOutput_;
    double setPoint_ = 0.0;
    double iTerm_ = 0.0;
    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;
    double outMin_ = 0.0;
    double outMax_ = 0.0;
    double outDuMax_ = 0.0;
};