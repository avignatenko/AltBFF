#pragma once

#include <cstdint>

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
    uint8_t positionFollowingEngage = 0;  // bit0, bit1, bit2  = axis1, axis2, axis3 (0/1)
    uint8_t reserved[7] = {0};
    Axis elevator;
    Axis aileron;
    Axis rudder;
};

struct CLReturn
{
    uint32_t packetID = 0;
    uint32_t lastInputPacketID = 0;
    uint8_t forceEnableStatus = 0;
    uint8_t driveAxis1Status = 0;
    uint8_t driveAxis2Status = 0;
    uint8_t driveAxis3Status = 0;
    uint8_t cardAutoCalibrationStatus = 0;
    uint8_t cardCondition = 0;
    uint8_t reserved[3] = {0};
    float axisElevatorPosition = 0;
    float axisAileronPosition = 0;
    float axisRudderPosition = 0;
    float axisElevatorTorque = 0;
    float axisAileronTorque = 0;
    float axisRudderTorque = 0;
    uint8_t reserved2[5] = {0};
    uint32_t packetIDRep = 0;
};

#pragma pack(pop)

static_assert(sizeof(CLInput) == 180, "CLInput size is not correct");

static_assert(sizeof(CLReturn) == 50, "CLReturn size is not correct");

}  // namespace bffcl
