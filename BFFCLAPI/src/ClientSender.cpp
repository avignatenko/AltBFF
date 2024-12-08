#include "ClientSender.h"

using namespace asio;
using namespace bffcl;

ClientSender::ClientSender(io_context& io, socket& socket, const std::string& addressRemote, int portRemote,
                           double timerInterval)
    : io_(io),
      socket_(socket),
      endpointRemote_(endpoint(address::from_string(addressRemote), portRemote)),
      timer_(io),
      timerInterval_(int(1000.0 / timerInterval))
{
    // set CL input defaults
    CLInput& input = lockInput();

    std::string localAddress = socket_.local_endpoint().address().to_string();
    localAddress.copy(input.feederIP, localAddress.size());
    input.returnPort = socket_.local_endpoint().port();
    unlockInput();

    // start sendering (fixme: move to start())
    timer_.expires_after(std::chrono::milliseconds(0));  // set default value to now
    send();
}

ClientSender::~ClientSender()
{
    stop();
}

void ClientSender::start() {}

void ClientSender::stop()
{
    io_.dispatch([this] { stopRequested_ = true; });
}

CLInput& ClientSender::lockInput()
{
    currentInputMutex_.lock();
    return currentInput_;
}

void ClientSender::unlockInput()
{
    currentInputMutex_.unlock();
}

void ClientSender::send()
{
    // reengage the timer
    timer_.expires_at(timer_.expiry() + timerInterval_);
    timer_.async_wait([this](const asio::error_code&) { doSend(); });
}

void ClientSender::doSend()
{
    if (stopRequested_) return;

    // do send
    CLInput& input = lockInput();
    input.packetID = packetId_++;
    std::shared_ptr<CLInput> clInput = std::make_shared<CLInput>(input);
    unlockInput();

    // clInput captured in lambda, so will be destroyed after handler completes
    socket_.async_send_to(buffer(clInput.get(), sizeof(CLInput)), endpointRemote_,
                          [clInput](const asio::error_code& error, std::size_t bytes_transferred) {});

    send();
}
