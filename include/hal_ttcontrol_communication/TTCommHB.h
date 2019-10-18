#include <hal_ttcontrol_communication/Can.h>

class TTCommHB
{
    Can _CanHandler;
    uint8_t _u8HbVal;
public:
    TTCommHB();
    void vSendHanshake();
    bool bWaitForHandshake();
    void vSendHB();
    int iHBReceived();
};