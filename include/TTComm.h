#pragma once
#include <Can.h>
#include <CartStatus.h>
#include <queue>

#define VELOCITIES_MASK 0x01
#define FRONT_ANGLE_MASK 0x02
#define BACK_ANGLE_MASK 0x04
#define PROTOCOL_MASK 0x08

class TTComm
{
    Can _CanHandler;
    uint8_t _u8HbVal; 
    std::queue<can_frame> _FramesQueue;
    CartStatus _FrontCart;
    CartStatus _BackCart;
    int iState;
public:
    TTComm();
    void vSendHanshake();
    bool bWaitForHandshake();
    void vSendHB();
    int iCorrectHBReceived();

    void vReceiveCanFrames();
    uint8_t u8ParseCanFrames();
    void vConfirmAdvantechReadiness();
    bool bWaritForTTControlReadiness();
    void vSendVelocities(float fFrontLeft, float fFrontRight, float fBackLeft, float fBackRight);
};