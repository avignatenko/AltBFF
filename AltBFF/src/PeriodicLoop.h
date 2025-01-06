#pragma once

#include <Utils/PeriodicTimer.h>

#include <asio.hpp>
#include <asio/io_context.hpp>

template <class S>
class PeriodicLoop2
{
public:
    PeriodicLoop2(double freq, asio::io_context& io, S& obj)
        : processTimer_(io, std::chrono::milliseconds(static_cast<int>(1000 / freq)))
    {
        processTimer_.wait([&obj] { obj.process(); });
    }

private:
    PeriodicTimer processTimer_;
};
