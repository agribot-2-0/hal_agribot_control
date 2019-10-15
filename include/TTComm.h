#pragma once
#include <Can.h>
#include <queue>

class TTComm
{
    Can _CanHandler;
    uint8_t _u8HbVal; 
    std::queue<can_frame> _VelocitiesFrames;
    std::queue<can_frame> _AnglesFrame;
    std::queue<can_frame> _ProtocolFrames;
public:
    TTComm();
    void vSendHanshake();
    bool bWaitForHandshake();
    void vSendHB();
    int iCorrectHBReceived();

    int iReceiveCanFrames();
    void vConfirmAdvantechReadiness();
};