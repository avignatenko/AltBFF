#pragma once

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
    UDPClient(const std::string& toAddress, int toPort, int fromPort);
    ~UDPClient();

    void start();
    void stop();

private:
    void run();

private:
    std::unique_ptr<std::thread> runner_;

    boost::asio::io_service io_;
    boost::asio::ip::udp::socket socket_;

    std::unique_ptr<ClientSender> sender_;
    std::unique_ptr<ClientReceiver> receiver_;
};
}  // namespace bffcl