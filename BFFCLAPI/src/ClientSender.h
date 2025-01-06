#pragma once

#include "CLStructures.h"

#include <Utils/ObjectPool.h>

#include <asio.hpp>
#include <asio/io_context.hpp>

namespace bffcl
{

class ClientSender
{
public:
    ClientSender(asio::io_context& io, asio::ip::udp::socket& socket, const std::string& addressRemote, int portRemote);
    ~ClientSender();

    CLInput& lockInput();

    void process();

private:
    void send();

private:
    asio::io_context& io_;
    asio::ip::udp::socket& socket_;
    asio::ip::udp::endpoint endpointRemote_;

    unsigned int packetId_ = 1;

    CLInput currentInput_;

    ObjectPool<CLInput> inputPool_;
};

}  // namespace bffcl
