#pragma once

#include <vector>
#include <optional>

// Simple double moving average with explicit computation of mean (O(N))
// Made to prevent accurary problems with usul increment computation
template <size_t N>
class MovingAverage
{
public:

    MovingAverage()
    {
        samples_.reserve(N);
    }

    void addSample(double sample)
    {
        if (samples_.size() < N)
            samples_.push_back(sample);
        else
        {
            samples_[nextSample_] = sample;
            nextSample_ = ++nextSample_ % N;
        }

        cachedAverage_ = std::nullopt;
    }

    // O(N) !!!
    double get()
    {
        if (cachedAverage_) return cachedAverage_.value();
        if (samples_.empty()) return 0.0;

        double total = 0.0;
        for (double sample : samples_)
            total += sample;
        total /= samples_.size();

        cachedAverage_ = total;

        return total;
    }

private:
    std::vector<double> samples_;
    int nextSample_ = 0;
    std::optional<double> cachedAverage_;
};

// N-day EMA
class ExponentialMovingAverage
{
public:

    ExponentialMovingAverage(int n): alpha_(2.0 / (n + 1))
    {
    }

    void addSample(double sample)
    {
        if (firstSample_)
        {
            average_ = lastAverage_ = sample;
            firstSample_ = false;
            return;
        }

        average_ = alpha_ * sample + (1 - alpha_) * lastAverage_;
        lastAverage_ = average_;
    }

    double get()
    {
        return average_;
    }

private:
    const double alpha_;

    bool firstSample_ = true;
    double average_ = 0.0;
    double lastAverage_ = 0.0;
};
