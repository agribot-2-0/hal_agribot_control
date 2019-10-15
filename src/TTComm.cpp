#include"../include/TTComm.h"

#define TTC_HANDSHAKE_ID 0x00EF64C8
#define TTC_RESPONSE_HANDSHAKE_ID 0x000EFC864
#define TTC_HB_ID 0x00FFDFC8
#define TTC_RESPONSE_HB_ID 0x00FFDF64
#define TTC_READINESS_CONFIRM_ID 0x000EFC864

TTComm::TTComm()
{
    _CanHandler.iInitialize("can0");
    _u8HbVal = 0;
}

void TTComm::vSendHanshake()
{
    //Set sending cycle to 1s:
    can_frame message;
    message.can_id = TTC_HANDSHAKE_ID;
    message.can_dlc = 8;
    message.data[0] = 0x10;
    message.data[1] = 0;
    message.data[2] = 0; //write
    message.data[3] = 0;
    message.data[4] = 0;
    message.data[5] = 0;
    message.data[6] = 0;
    message.data[7] = 0;
    _CanHandler.iSendFrame(&message);
}

bool TTComm::bWaitForHandshake()
{
    can_frame message;
    while(_CanHandler.bReceiveFrame(&message))
    {
        if(((message.can_id)&(0xFFFF)) == (TTC_RESPONSE_HANDSHAKE_ID&(0xFFFF)))
        {
            return true;
        }
    }
    return false;
}

void TTComm::vSendHB()
{
    //Set sending cycle to 1s:
    can_frame message;
    message.can_id = TTC_HB_ID;
    message.can_dlc = 8;
    message.data[0] = (this->_u8HbVal);
    message.data[1] = 0;
    message.data[2] = 0; //write
    message.data[3] = 0;
    message.data[4] = 0;
    message.data[5] = 0;
    message.data[6] = 0;
    message.data[7] = 0;
    _CanHandler.iSendFrame(&message);
    std::cout << "Sending: ";
    std::cout << (int)_u8HbVal << std::endl;
}

int TTComm::iCorrectHBReceived()
{
    can_frame message;

    while(_CanHandler.bReceiveFrame(&message))
    {
        if(((message.can_id)&(0xFFFF)) == (TTC_RESPONSE_HB_ID&(0xFFFF)))
        {
            if(_u8HbVal+1 == message.data[0])
            {
                std::cout << "Received: ";
                std::cout << (int)message.data[0] << std::endl;
                _u8HbVal+=2;
                return 0;
            }
            else //wrong hb value
            {
                return -2;
            }

        }
        std::cout << "rec frame" << std::endl;
    }
    return -1;
}

int TTComm::iWaitForTTControlReadiness()
{
    can_frame message;
    while(_CanHandler.bReceiveFrame(&message))
    {
        if(TTC_READINESS_CONFIRM_ID&0xFFFFFFF == ((message.can_id)&(0xFFFFFFF)) && message.data[0] == 0x20)
        {
            if(message.data[1]==0x01)
            {
                return 0;
            }
        }
        else
        {
            return -2;
        }
    }
    return -1;
}

bool TTComm::bConfirmAdvantechReaqdiness()
{
    return true;
}