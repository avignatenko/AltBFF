#pragma once

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <thread>

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

class ClientSender;
class ClientReceiver;

class UDPClient
{
public:
    UDPClient(const std::string& toAddress, int toPort, const std::string& fromAddress, int fromPort);
    ~UDPClient();

    void start();
    void stop();

    CLInput& lockInput();
    void unlockInput();

private:
    void run();

private:
    std::unique_ptr<std::thread> runner_;

    boost::asio::io_service io_;
    boost::asio::ip::udp::socket socket_;

    std::unique_ptr<ClientSender> sender_;
    std::unique_ptr<ClientReceiver> receiver_;
};
}  // namespace bffcl