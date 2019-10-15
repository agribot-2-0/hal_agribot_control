#pragma once
#include "../include/Can.h"

class TTComm
{
    Can _CanHandler;
    uint8_t _u8HbVal; 
public:
    TTComm();
    void vSendHanshake();
    bool bWaitForHandshake();
    void vSendHB();
    int iCorrectHBReceived();

    int iWaitForTTControlReadiness();
    bool bConfirmAdvantechReaqdiness();
};