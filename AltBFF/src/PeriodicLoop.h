#pragma once

#include <Utils/PeriodicTimer.h>

#include <asio.hpp>
#include <asio/io_context.hpp>

template <class F>
class PeriodicLoop
{
public:
    PeriodicLoop(double freq, asio::io_context& io, F&& func)
        : io_(io), processTimer_(io, std::chrono::milliseconds(static_cast<int>(1000 / freq)))
    {
        processTimer_.wait(std::move(func));
    }

private:
    asio::io_context& io_;
    PeriodicTimer processTimer_;
};
