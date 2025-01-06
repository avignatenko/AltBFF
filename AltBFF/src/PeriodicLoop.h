#pragma once

#include <Utils/PeriodicTimer.h>

#include <asio.hpp>
#include <asio/io_context.hpp>

class PeriodicLoop
{
public:
    PeriodicLoop(double freq, asio::io_context& io)
        : io_(io), processTimer_(io, std::chrono::milliseconds(static_cast<int>(1000 / freq)))
    {
        processTimer_.wait([this] { process(); });
    }

protected:
    virtual void process() = 0;

private:
    asio::io_context& io_;
    PeriodicTimer processTimer_;
};
