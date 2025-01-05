#pragma once

#include <asio/io_context.hpp>
#include <asio/steady_timer.hpp>

class PeriodicTimer
{
public:
    using Duration = typename std::chrono::steady_clock::duration;

    PeriodicTimer(asio::io_context& io, Duration duration) : timer_(io), duration_(duration) {}

    template <typename F>
    void wait(F&& func)
    {
        timer_.expires_after(std::chrono::milliseconds(0));
        timeoutHandler(func);
    }

private:
    template <typename F>
    void timeoutHandler(F&& func)
    {
        timer_.async_wait(
            [this, f = std::move(func)](std::error_code ec)
            {
                f();
                timer_.expires_at(timer_.expiry() + duration_);
                timeoutHandler(std::move(f));
            });
    }

private:
    asio::steady_timer timer_;
    Duration duration_;
};
