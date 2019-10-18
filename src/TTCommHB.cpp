#include<TTCommHB.h>

#define PROTOCOL_RESPONSE_FRAME_ID 0x00EF64C8
#define PROTOCOL_FRAME_ID 0x000EFC864
#define TTC_HB_ID 0x00FFDFC8
#define TTC_RESPONSE_HB_ID 0x00FFDF64

TTCommHB::TTCommHB()
{
    _CanHandler.iInitialize("can0");
    _u8HbVal = 0;
}

void TTCommHB::vSendHanshake()
{
    can_frame message;
    message.can_id = PROTOCOL_RESPONSE_FRAME_ID;
    message.can_dlc = 8;
    message.data[0] = 0x10;
    message.data[1] = 0;
    message.data[2] = 0;
    message.data[3] = 0;
    message.data[4] = 0;
    message.data[5] = 0;
    message.data[6] = 0;
    message.data[7] = 0;
    _CanHandler.iSendFrame(&message);
}

bool TTCommHB::bWaitForHandshake()
{
    can_frame message;
    while(_CanHandler.bReceiveFrame(&message))
    {
        if(((message.can_id)&(0xFFFF)) == (PROTOCOL_FRAME_ID&(0xFFFF)))
        {
            return true;
        }
    }
    return false;
}

void TTCommHB::vSendHB()
{
    can_frame message;
    message.can_id = TTC_HB_ID;
    message.can_dlc = 8;
    message.data[0] = (this->_u8HbVal);
    message.data[1] = 0;
    message.data[2] = 0;
    message.data[3] = 0;
    message.data[4] = 0;
    message.data[5] = 0;
    message.data[6] = 0;
    message.data[7] = 0;
    _CanHandler.iSendFrame(&message);
}

int TTCommHB::iHBReceived()
{
    can_frame message;
    while(_CanHandler.bReceiveFrame(&message))
    {
        if(((message.can_id)&(0xFFFF)) == (TTC_RESPONSE_HB_ID&(0xFFFF)))
        {
            if(_u8HbVal+1 == message.data[0])
            {
                _u8HbVal+=2;
                return 0;
            }
            else
            {
                return -2;
            }
        }
    }
    return -1;
}
