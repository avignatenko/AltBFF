// AlfBFF.cpp : Defines the entry point for the application.
//

#include "UDPClient.h"

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>

#include <promise-cpp/promise.hpp>

using namespace boost::asio;
using namespace boost::asio::ip;

namespace promise
{
template <typename RESULT>
inline void setPromise(Defer d, boost::system::error_code err, const char* errorString, const RESULT& result)
{
    if (err)
    {
        spdlog::error("{}: {}", errorString, err.message());
        d.reject(err);
    }
    else
        d.resolve(result);
}

}  // namespace promise

namespace bffcl
{
const std::chrono::milliseconds kTimerInterval = boost::asio::chrono::milliseconds(20);

class ClientSender
{
public:
    ClientSender(io_service& io, udp::socket& socket, const std::string& addressRemote, int portRemote)
        : io_(io),
          socket_(socket),
          endpointRemote_(udp::endpoint(address::from_string(addressRemote), portRemote)),
          timer_(io, kTimerInterval)
    {
        // set CL input defaults
        {
            std::scoped_lock lock(currentInputMutex_);

            std::string localAddress = socket_.local_endpoint().address().to_string();
            localAddress.copy(currentInput_.feederIP, localAddress.size());
            currentInput_.returnPort = socket_.local_endpoint().port();

            // currentInput_.elevator.fixedForce = 10;
        }

        timer_.async_wait([this](const boost::system::error_code&) { onTimer(); });
    }
    ~ClientSender() { stop(); }

    void start() {}

    void stop()
    {
        io_.dispatch([this] { stopRequested_ = true; });
    }

    CLInput& lockInput()
    {
        currentInputMutex_.lock();
        return currentInput_;
    }

    void unlockInput() { currentInputMutex_.unlock(); }

private:
    void onTimer()
    {
        if (stopRequested_) return;

        // do send
        currentInput_.packetID = packetId_++;
        std::shared_ptr<CLInput> clInput = std::make_shared<CLInput>(currentInput_);

        socket_.async_send_to(buffer(clInput.get(), sizeof(CLInput)), endpointRemote_,
                              [clInput](const boost::system::error_code& error, std::size_t bytes_transferred) {});

        // reengage the timer
        timer_.expires_at(timer_.expiry() + kTimerInterval);
        timer_.async_wait([this](const boost::system::error_code&) { onTimer(); });
    }

private:
    io_service& io_;
    udp::socket& socket_;
    udp::endpoint endpointRemote_;

    steady_timer timer_;
    bool stopRequested_ = false;

    unsigned int packetId_ = 1;

    CLInput currentInput_;
    std::mutex currentInputMutex_;

};  // namespace bffcl

class ClientReceiver
{
public:
    ClientReceiver(io_service& io, udp::socket& socket) : io_(io), socket_(socket) {}
    ~ClientReceiver() { stop(); }
    void start() {}
    void stop() {}

private:
    io_service& io_;
    udp::socket& socket_;
};

UDPClient::UDPClient(const std::string& toAddress, int toPort, const std::string& fromAddress, int fromPort)
    : socket_(io_)
{
    udp::endpoint send_endpoint(address::from_string(fromAddress), fromPort);
    socket_.open(send_endpoint.protocol());
    socket_.bind(send_endpoint);

    sender_ = std::make_unique<ClientSender>(io_, socket_, toAddress, toPort);
    receiver_ = std::make_unique<ClientReceiver>(io_, socket_);

    runner_ = std::make_unique<std::thread>(&UDPClient::run, this);
}

bffcl::UDPClient::~UDPClient()
{
    sender_->stop();
    receiver_->stop();

    io_.post([this] { socket_.close(); });

    runner_->join();
}

void UDPClient::run()
{
    io_.run();
}

CLInput& UDPClient::lockInput()
{
    return sender_->lockInput();
}

void UDPClient::unlockInput()
{
    return sender_->unlockInput();
}

}  // namespace bffcl