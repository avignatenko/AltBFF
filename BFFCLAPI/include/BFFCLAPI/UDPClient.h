#pragma once

#include "CLStructures.h"

#include <asio.hpp>
#include <asio/io_context.hpp>
#include <thread>

namespace bffcl
{
class ClientSender;
class ClientReceiver;

class UDPClient
{
public:
    struct Settings
    {
        std::string toAddress;
        int toPort;
        std::string fromAddress;
        int fromPort;

        double sendFreq = 50;  // 50hz
    };

    UDPClient(const Settings& settings, asio::io_context& io);
    ~UDPClient();

    void start();
    void stop();

    CLInput& lockInput();

    const CLReturn& lockOutput();

private:
    using io_context = asio::io_context;
    using socket = asio::ip::udp::socket;

    io_context& io_;
    socket socket_;

    std::unique_ptr<ClientSender> sender_;
    std::unique_ptr<ClientReceiver> receiver_;
};
}  // namespace bffcl
