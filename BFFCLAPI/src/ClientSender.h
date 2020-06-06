#pragma once

#include "CLStructures.h"

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>

#include <mutex>

namespace bffcl
{

class ClientSender
{
public:
    using io_context = boost::asio::io_context;
    using socket = boost::asio::ip::udp::socket;
    using endpoint = boost::asio::ip::udp::endpoint;
    using address = boost::asio::ip::address;
    using steady_timer = boost::asio::steady_timer;

    ClientSender(io_context& io, socket& socket, const std::string& addressRemote, int portRemote, double freq);
    ~ClientSender();

    void start();
    void stop();
    CLInput& lockInput();
    void unlockInput();

private:

  
    void send();
    void doSend();

private:
    io_context& io_;
    socket& socket_;

    endpoint endpointRemote_;

    std::chrono::milliseconds timerInterval_;
    steady_timer timer_;
    bool stopRequested_ = false;

    unsigned int packetId_ = 1;

    CLInput currentInput_;
    std::mutex currentInputMutex_;
};

}  // namespace bffcl