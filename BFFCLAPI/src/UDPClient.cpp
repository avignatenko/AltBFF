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
          timer_(io)
    {
        // set CL input defaults
        CLInput& input = lockInput();

        std::string localAddress = socket_.local_endpoint().address().to_string();
        localAddress.copy(input.feederIP, localAddress.size());
        input.returnPort = socket_.local_endpoint().port();
        unlockInput();

        // start sendering (fixme: move to start())
        send();
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
    void send()
    {
        // reengage the timer
        timer_.expires_at(timer_.expiry() + kTimerInterval);
        timer_.async_wait([this](const boost::system::error_code&) { doSend(); });
    }

    void doSend()
    {
        if (stopRequested_) return;

        // do send
        CLInput& input = lockInput();
        input.packetID = packetId_++;
        std::shared_ptr<CLInput> clInput = std::make_shared<CLInput>(input);
        unlockInput();

        // clInput captured in lambda, so will be destroyed after handler completes
        socket_.async_send_to(buffer(clInput.get(), sizeof(CLInput)), endpointRemote_,
                              [clInput](const boost::system::error_code& error, std::size_t bytes_transferred) {});

        send();
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
    void stop()
    {
        io_.dispatch([this] { stopRequested_ = true; });
    }

    CLReturn& lockOutput()
    {
        currentOutputMutex_.lock();
        return currentOutput_;
    }

    void unlockOutput() { currentOutputMutex_.unlock(); }

private:
    void receive()
    {
        // do receive

        std::shared_ptr<CLReturn> clOutput = std::make_shared<CLReturn>();
        std::shared_ptr<udp::endpoint> senderEndpoint = std::make_shared<udp::endpoint>();
        socket_.async_receive_from(
            boost::asio::buffer(clOutput.get(), sizeof(CLReturn)), *senderEndpoint,
            [this, clOutput, senderEndpoint](const boost::system::error_code& error, std::size_t reply_length) {
                // copy to destination
                CLReturn& currentOutput = lockOutput();
                currentOutput = *clOutput;
                unlockOutput();

                if (!stopRequested_) receive();
            });
    }

private:
    io_service& io_;
    udp::socket& socket_;

    unsigned int packetId_ = 1;
    bool stopRequested_ = false;

    CLReturn currentOutput_;
    std::mutex currentOutputMutex_;
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

const CLReturn& UDPClient::lockOutput()
{
    return receiver_->lockOutput();
}
void UDPClient::unlockOutput()
{
    return receiver_->unlockOutput();
}

}  // namespace bffcl