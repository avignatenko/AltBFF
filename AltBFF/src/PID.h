#pragma once

#include <spdlog/spdlog.h>

#include <optional>
#include <algorithm>
#include <tuple>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

// after http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
class PIDController
{
public:

    PIDController(double kp, double ki, double kd, double min, double max, double sampleTimeMs)
    {
        kp_ = kp;
        ki_ = ki * sampleTimeMs;
        kd_ = kd / sampleTimeMs;
        setOutputLimits(min, max);
    }


    void compute()
    {
        /*Compute all the working error variables*/
        double error = setPoint_ - input_;
        iTerm_ += (ki_ * error);
        iTerm_ = std::clamp(iTerm_, outMin_, outMax_);

        double dInput = (input_ - lastInput.value());
        dInputAccum_(dInput);
 
        /*Compute PID output*/
        output_ = kp_ * error + iTerm_ - kd_ * boost::accumulators::rolling_mean(dInputAccum_);
        output_ = std::clamp(output_, outMin_, outMax_);

        /*Remember some variables for next time*/
        lastInput = input_;
    }

    std::tuple<double, double, double, double, double, double> dumpInternals()
    {
        return { setPoint_,  input_, output_, kp_ * (setPoint_ - input_), iTerm_, - kd_ * boost::accumulators::rolling_mean(dInputAccum_) };
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

    using Accumulator = boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::rolling_mean> >;
    using AccParams = boost::accumulators::tag::rolling_window;

    Accumulator dInputAccum_ = Accumulator(AccParams::window_size = 30);
};