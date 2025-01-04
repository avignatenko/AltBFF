// AlfBFF.cpp : Defines the entry point for the application.
//

#include "ClientReceiver.h"
#include "ClientSender.h"

#include "UDPClient.h"

#include <asio.hpp>
#include <asio/io_context.hpp>

#include <spdlog/spdlog.h>

using address = asio::ip::address;
using endpoint = asio::ip::udp::endpoint;

using namespace bffcl;

UDPClient::UDPClient(const Settings& settings, asio::io_context& io) : socket_(io), io_(io)
{
    spdlog::info("Opening socket, Endpoints: to {}:{}, from {}:{}", settings.toAddress, settings.toPort,
                 settings.fromAddress, settings.fromPort);
    endpoint send_endpoint(address::from_string(settings.fromAddress), settings.fromPort);
    socket_.open(send_endpoint.protocol());
    socket_.bind(send_endpoint);
    spdlog::info("Socket opened successfully");

    sender_ = std::make_unique<ClientSender>(io_, socket_, settings.toAddress, settings.toPort, settings.sendFreq);
    receiver_ = std::make_unique<ClientReceiver>(io_, socket_);
}

bffcl::UDPClient::~UDPClient()
{
    socket_.close();
}

CLInput& UDPClient::lockInput()
{
    return sender_->lockInput();
}

const CLReturn& UDPClient::lockOutput()
{
    return receiver_->lockOutput();
}
