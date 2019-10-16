#pragma once
#include <Can.h>
#include <CartStatus.h>
#include <queue>

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
    int iParseCanFrames();
    void vConfirmAdvantechReadiness();
    void vSendVelocities(float fLeftVelo, float fRightVelo);
};