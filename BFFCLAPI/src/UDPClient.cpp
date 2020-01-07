// AlfBFF.cpp : Defines the entry point for the application.
//

#include "UDPClient.h"

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>

using namespace boost::asio;
using namespace boost::asio::ip;

namespace bffcl
{
#pragma pack(push, 1)

struct Axis
{
    float fixedForce = 0;
    float springForce = 0;
    uint16_t frictionCoeff = 0;
    uint16_t dumpingCoeff = 0;
    uint16_t vibrationCh1Hz = 0;
    uint16_t vibrationCh1Amp = 0;
    uint16_t vibrationCh2Hz = 0;
    uint16_t vibrationCh2Amp = 0;
    uint16_t vibrationCh3Hz = 0;
    uint16_t vibrationCh3Amp = 0;
    uint16_t positionFollowingP = 0;
    uint16_t positionFollowingI = 0;
    uint16_t positionFollowingD = 0;
    float positionFollowingSetPoint = 0;
    uint8_t breakoutForce = 0;
    uint8_t breakoutAmplitude = 0;
    uint8_t reserved[14] = {0};
};

struct CLInput
{
    uint32_t packetID = 0;
    uint8_t loadingEngage = 0;
    char feederIP[15] = {0};
    uint16_t returnPort = 0;
    uint8_t positionFollowingEngage = 0;
    uint8_t reserved[7] = {0};
    Axis elevator;
    Axis aileron;
    Axis rudder;
};
#pragma pack(pop)

static_assert(sizeof(CLInput) == 180, "CLInput size is not correct");

class ClientSender
{
public:
    ClientSender(io_service& io, udp::socket& socket, const std::string& addressRemote, int portRemote)
        : io_(io),
          socket_(socket),
          endpointRemote_(udp::endpoint(address::from_string(addressRemote), portRemote)),
          timer_(io, kTimerInterval)
    {
        timer_.async_wait([this](const boost::system::error_code&) { onTimer(); });
    }
    ~ClientSender() { stop(); }

    void start() {}
    void stop()
    {
        io_.dispatch([this] { stopRequested_ = true; });
    }

    void onTimer()
    {
        if (stopRequested_) return;

        // do send
        std::shared_ptr<CLInput> clInput = std::make_unique<CLInput>();
        socket_.async_send_to(buffer(clInput.get(), sizeof(CLInput)), endpointRemote_,
                              [clInput](const boost::system::error_code& error, std::size_t bytes_transferred) {});

        // reengage the timer
        timer_.expires_at(timer_.expiry() + kTimerInterval);
        timer_.async_wait([this](const boost::system::error_code&) { onTimer(); });
    }

private:
    const std::chrono::milliseconds kTimerInterval = boost::asio::chrono::milliseconds(20);
    io_service& io_;
    udp::socket& socket_;
    udp::endpoint endpointRemote_;

    steady_timer timer_;
    bool stopRequested_ = false;

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

UDPClient::UDPClient(const std::string& toAddress, int toPort, int fromPort) : socket_(io_)
{
    udp::endpoint send_endpoint(udp::v4(), fromPort);
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

    socket_.close();

    runner_->join();
}

void UDPClient::run()
{
    io_.run();
}

}  // namespace bffcl