#pragma once

#include "CLStructures.h"

#include <asio.hpp>
#include <asio/io_context.hpp>

#include <mutex>

namespace bffcl
{

struct CLReturn;

class ClientReceiver
{
public:
    using io_context = asio::io_context;
    using socket = asio::ip::udp::socket;
    using endpoint = asio::ip::udp::endpoint;

    ClientReceiver(io_context& io, socket& socket);
    ~ClientReceiver();

    void start();
    void stop();

    CLReturn& lockOutput();

    void unlockOutput();

private:
    void receive();

private:
    io_context& io_;
    socket& socket_;

    unsigned int packetId_ = 1;
    bool stopRequested_ = false;

    CLReturn currentOutput_;
    std::mutex currentOutputMutex_;
};

}  // namespace bffcl