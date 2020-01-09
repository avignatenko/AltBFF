#pragma once

#include "CLStructures.h"

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
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
    };

    UDPClient(const Settings& settings);
    ~UDPClient();

    void start();
    void stop();

    CLInput& lockInput();
    void unlockInput();

    const CLReturn& lockOutput();
    void unlockOutput();

private:
    std::unique_ptr<std::thread> runner_;

    using io_context = boost::asio::io_context;
    using socket = boost::asio::ip::udp::socket;

    io_context io_;
    socket socket_;

    std::unique_ptr<ClientSender> sender_;
    std::unique_ptr<ClientReceiver> receiver_;
};
}  // namespace bffcl