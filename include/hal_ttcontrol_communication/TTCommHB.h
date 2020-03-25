#pragma once

#include <agribot_common/Can.h>

#define CORRECT_HB_RECEIVED 0
#define WRONG_HB_RECEIVED -2

class TTCommHB
{
    Can _CanHandler;
    uint8_t _u8HbVal;
 public:
    explicit TTCommHB(char* pchCanChannel);
    void vSendHanshake();
    bool bIsHandshakeReceived();
    void vSendHB();
    int iHBReceived();
};
