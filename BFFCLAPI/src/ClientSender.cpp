#include "ClientSender.h"

using namespace asio;
using namespace bffcl;

ClientSender::ClientSender(io_context& io, ip::udp::socket& socket, const std::string& addressRemote, int portRemote)
    : io_(io), socket_(socket), endpointRemote_(ip::udp::endpoint(ip::address::from_string(addressRemote), portRemote))
{
    // set CL input defaults
    CLInput& input = lockInput();

    std::string localAddress = socket_.local_endpoint().address().to_string();
    localAddress.copy(input.feederIP, localAddress.size());
    input.returnPort = socket_.local_endpoint().port();
}

ClientSender::~ClientSender() {}

CLInput& ClientSender::lockInput()
{
    return currentInput_;
}

void ClientSender::send()
{
    // do send
    CLInput& input = lockInput();
    input.packetID = packetId_++;

    std::shared_ptr<CLInput> clInput = inputPool_.aquire();
    *clInput = input;

    // clInput captured in lambda, so will be destroyed after handler completes
    socket_.async_send_to(buffer(clInput.get(), sizeof(CLInput)), endpointRemote_,
                          [this, clInput](const asio::error_code& error, std::size_t bytes_transferred) mutable
                          { inputPool_.release(clInput); });
}

void ClientSender::process()
{
    send();
}
