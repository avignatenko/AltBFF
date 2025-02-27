#include "ClientReceiver.h"

using namespace bffcl;
using namespace asio;

ClientReceiver::ClientReceiver(io_context& io, ip::udp::socket& socket) : io_(io), socket_(socket)
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
    auto clOutput = inputPool_.aquire();

    socket_.async_receive_from(asio::buffer(clOutput.get(), sizeof(CLReturn)), senderEndpoint_,
                               [this, clOutput](const asio::error_code& error, std::size_t reply_length)
                               {
                                   // copy to destination
                                   CLReturn& currentOutput = lockOutput();
                                   currentOutput = *clOutput;

                                   inputPool_.release(clOutput);

                                   receive();
                               });
}
