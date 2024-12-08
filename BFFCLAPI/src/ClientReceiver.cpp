#include "ClientReceiver.h"

using namespace bffcl;

ClientReceiver::ClientReceiver(io_context& io, socket& socket) : io_(io), socket_(socket)
{
    // fixme: move to start
    receive();
}

ClientReceiver::~ClientReceiver() {}

CLReturn& ClientReceiver::lockOutput()
{
    return currentOutput_;
}

void ClientReceiver::receive()
{
    // do receive
    auto clOutput = std::make_shared<CLReturn>();
    auto senderEndpoint = std::make_shared<endpoint>();

    socket_.async_receive_from(asio::buffer(clOutput.get(), sizeof(CLReturn)), *senderEndpoint,
                               [this, clOutput, senderEndpoint](const asio::error_code& error, std::size_t reply_length)
                               {
                                   // copy to destination
                                   CLReturn& currentOutput = lockOutput();
                                   currentOutput = *clOutput;

                                   receive();
                               });
}
