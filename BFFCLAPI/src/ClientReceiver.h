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
    ClientReceiver(asio::io_context& io, asio::ip::udp::socket& socket);
    ~ClientReceiver();

    CLReturn& lockOutput();

private:
    void receive();

private:
    asio::io_context& io_;
    asio::ip::udp::socket& socket_;

    unsigned int packetId_ = 1;

    CLReturn currentOutput_;
};

}  // namespace bffcl
