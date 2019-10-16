#include<TTComm.h>

#define TTC_HANDSHAKE_ID 0x00EF64C8
#define TTC_RESPONSE_HANDSHAKE_ID 0x000EFC864
#define TTC_HB_ID 0x00FFDFC8
#define TTC_RESPONSE_HB_ID 0x00FFDF64
#define TTC_READINESS_CONFIRM_ID 0x000EFC864
#define ADVANTECH_READY_ID 0x00EF64C8
#define VELO_FRAME_ID 0x00FFD064
#define ANGLE_FRAME_ID 0xDDDD
#define PROTOCOL_FRAME_ID 0x000EFC864

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
                _u8HbVal+=2;
                return 0;
            }
            else //wrong hb value
            {
                return -2;
            }
        }
    }
    return -1;
}

void TTComm::iReceiveCanFrames()
{
    can_frame message;
    while(_CanHandler.bReceiveFrame(&message))
    {
        _FramesQueue.push(message);
    }
}

int TTComm::iParseCanFrames()
{
    can_frame message;
    while(!_FramesQueue.empty())
    {
        message = _FramesQueue.front();
        if((VELO_FRAME_ID&(0xFFFFFF)) == ((message.can_id)&(0xFFFFFF)))
        {
            _FrontCart.vSetLeftTrackVelo(((float)(((uint16_t)(message.data[0]&0x7F))<<8 | message.data[1]))*0.01);
            if(message.data[0]&0x80)
            {
                _FrontCart.vSetLeftTrackVelo(-_FrontCart.fGetLeftTrackVelo());
            }
            _FrontCart.vSetRightTrackVelo(((float)(((uint16_t)(message.data[2]&0x7F))<<8 | message.data[3]))*0.01);
            if(message.data[0]&0x80)
            {
                _FrontCart.vSetRightTrackVelo(-_FrontCart.fGetRightTrackVelo());
            }
            _BackCart.vSetLeftTrackVelo(((float)(((uint16_t)(message.data[4]&0x7F))<<8 | message.data[5]))*0.01);
            if(message.data[4]&0x80)
            {
                _BackCart.vSetLeftTrackVelo(-_BackCart.fGetLeftTrackVelo());
            }
            _BackCart.vSetRightTrackVelo(((float)(((uint16_t)(message.data[6]&0x7F))<<8 | message.data[7]))*0.01);
            if(message.data[6]&0x80)
            {
                _BackCart.vSetRightTrackVelo(-_BackCart.fGetRightTrackVelo());
            }
        }
        if((ANGLE_FRAME_ID&(0xFFFFFF)) == ((message.can_id)&(0xFFFFFF)))
        {
            
        }
        if((PROTOCOL_FRAME_ID&(0xFFFFFF)) == ((message.can_id)&(0xFFFFFF)))
        {
            
        }
        _FramesQueue.pop();
    }
    return 0;
}


void TTComm::vConfirmAdvantechReadiness()
{
    //Set sending cycle to 1s:
    can_frame message;
    message.can_id = ADVANTECH_READY_ID;
    message.can_dlc = 8;
    message.data[0] = 0x20;
    message.data[1] = 0x64;
    message.data[2] = 0;
    message.data[3] = 0;
    message.data[4] = 0;
    message.data[5] = 0;
    message.data[6] = 0;
    message.data[7] = 0;
    _CanHandler.iSendFrame(&message);
}


/*       std::cout << std::hex << (int)message.can_id << std::endl; 
        if((TTC_READINESS_CONFIRM_ID&(0xFFFFFF)) == ((message.can_id)&(0xFFFFFF)))
        {
            std::cout << "ok" << std::endl;
            if(message.data[1]==0x01 && message.data[0] == 0x20)
            {
                std::cout << "o2k" << std::endl;    
            }
            else
            {
            }
        }
 */