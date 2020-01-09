// AlfBFF.cpp : Defines the entry point for the application.
//

#include "ClientReceiver.h"
#include "ClientSender.h"

#include "UDPClient.h"

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>

#include <spdlog/spdlog.h>

using address = boost::asio::ip::address;
using endpoint = boost::asio::ip::udp::endpoint;

using namespace bffcl;

UDPClient::UDPClient(const Settings& settings)
    : socket_(io_)
{
    spdlog::info("Opening socket, Endpoints: to {}:{}, from {}:{}", settings.toAddress, settings.toPort,
                 settings.fromAddress, settings.fromPort);
    endpoint send_endpoint(address::from_string(settings.fromAddress), settings.fromPort);
    socket_.open(send_endpoint.protocol());
    socket_.bind(send_endpoint);
    spdlog::info("Socket opened successfully");

    sender_ = std::make_unique<ClientSender>(io_, socket_, settings.toAddress, settings.toPort);
    receiver_ = std::make_unique<ClientReceiver>(io_, socket_);

    runner_ = std::make_unique<std::thread>([this] {
        spdlog::info("Starting iocontext loop");
        io_.run();
        spdlog::info("iocontext finished");
    });
}

bffcl::UDPClient::~UDPClient()
{
    sender_->stop();
    receiver_->stop();

    io_.post([this] { socket_.close(); });

    runner_->join();
}

CLInput& UDPClient::lockInput()
{
    return sender_->lockInput();
}

void UDPClient::unlockInput()
{
    return sender_->unlockInput();
}

const CLReturn& UDPClient::lockOutput()
{
    return receiver_->lockOutput();
}
void UDPClient::unlockOutput()
{
    return receiver_->unlockOutput();
}
