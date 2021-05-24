#pragma once

class RateLimiter
{
public:
    RateLimiter(double fallingRate, double risingRate, double input, double sampleTimeMs)
    {
        output_ = input;
        sampleTimeMs_ = sampleTimeMs;
        fallingRateOrig_ = fallingRate;
        risingRateOrig_ = risingRate;
        updateRate();
    }

    void setInput(double input) { input_ = input; }

    double getInput() const { return input_; }

    void setRate(double fallingRate, double risingRate)
    {
        fallingRateOrig_ = fallingRate;
        risingRateOrig_ = risingRate;
        updateRate();
    }
    
    void setSampleTimeMs(double sampleTimeMs)
    {
        sampleTimeMs_ = sampleTimeMs;
        updateRate();
    }

    void process()
    {
        lastOutput_ = output_;

        double rate = input_ - lastOutput_;

        if (rate > risingRate_)
            output_ = lastOutput_ + risingRate_;
        else if (rate < fallingRate_)
            output_ = lastOutput_ + fallingRate_;
        else
            output_ = input_;
    }

    double getOutput() { return output_; }

private:

    void updateRate()
    {
        fallingRate_ = fallingRateOrig_ * getSampleTimeSec();
        risingRate_ = risingRateOrig_ * getSampleTimeSec();
    }

private:
    double getSampleTimeSec() const { return sampleTimeMs_ / 1000.0; }

    double sampleTimeMs_ = 0.0;
    double input_ = 0.0;
    double lastOutput_ = 0.0;
    double output_ = 0.0;

    double fallingRateOrig_ = 0.0;
    double risingRateOrig_ = 0.0;

    double fallingRate_ = 0.0;
    double risingRate_ = 0.0;
};