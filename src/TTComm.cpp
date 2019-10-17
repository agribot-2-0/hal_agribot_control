#include<TTComm.h>

#define PROTOCOL_FRAME_ID 0x000EFC864
#define PROTOCOL_RESPONSE_FRAME_ID 0x00EF64C8
#define TTC_HB_ID 0x00FFDFC8
#define TTC_RESPONSE_HB_ID 0x00FFDF64
#define VELO_FRAME_ID 0x00FFD064
#define FRONT_ANGLE_ID 0xaa20
#define BACK_ANGLE_ID 0xaa21
#define VELOCITY_CONTROL_FRAME_ID 0x00FFD0C8

TTComm::TTComm()
{
    _CanHandler.iInitialize("can0");
    _u8HbVal = 0;
}

void TTComm::vSendHanshake()
{
    //Set sending cycle to 1s:
    can_frame message;
    message.can_id = PROTOCOL_RESPONSE_FRAME_ID;
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
        if(((message.can_id)&(0xFFFF)) == (PROTOCOL_FRAME_ID&(0xFFFF)))
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

void TTComm::vReceiveCanFrames()
{
    can_frame message;
    while(_CanHandler.bReceiveFrame(&message))
    {
        _FramesQueue.push(message);
    }
}

uint8_t TTComm::u8ParseCanFrames()
{
    uint8_t u8Status=0;
    can_frame message;
    while(!_FramesQueue.empty())
    {
        message = _FramesQueue.front();
        if((VELO_FRAME_ID&(0xFFFFFF)) == ((message.can_id)&(0xFFFFFF)))
        {
            _Cart.vSetFrontLinear(((float)(((uint16_t)(message.data[0]&0x7F))<<8 | message.data[1]))*0.01);
            if(message.data[0]&0x80)
            {
                _Cart.vSetFrontLinear(-_Cart.fGetFrontLinear());
            }
            _Cart.vSetFrontAngular(((float)(((uint16_t)(message.data[2]&0x7F))<<8 | message.data[3]))*0.01);
            if(message.data[0]&0x80)
            {
                _Cart.vSetFrontAngular(-_Cart.fGetFrontAngular());
            }
            _Cart.vSetBackLinear(((float)(((uint16_t)(message.data[4]&0x7F))<<8 | message.data[5]))*0.01);
            if(message.data[4]&0x80)
            {
                _Cart.vSetBackLinear(-_Cart.fGetBackLinear());
            }
            _Cart.vSetBackAngular(((float)(((uint16_t)(message.data[6]&0x7F))<<8 | message.data[7]))*0.01);
            if(message.data[6]&0x80)
            {
                _Cart.vSetBackAngular(-_Cart.fGetBackAngular());
            }
            u8Status|=VELOCITIES_MASK;
        }
        if((FRONT_ANGLE_ID&(0xFFFF)) == ((message.can_id)&(0xFFFF)))
        {
            _fFrontCartAngle = ((float)(((uint32_t)message.data[2])<<16 | ((uint32_t)message.data[1])<<8 
            | ((uint32_t)message.data[0])))*0.216-180;
            u8Status|=FRONT_ANGLE_MASK;
        }
        if((BACK_ANGLE_ID&(0xFFFF)) == ((message.can_id)&(0xFFFF)))
        {
            _fBackCartAngle = ((float)(((uint32_t)message.data[2])<<16 | ((uint32_t)message.data[1])<<8 
            | ((uint32_t)message.data[0])))*0.216-180;
            u8Status|=BACK_ANGLE_MASK;
        }
        if((PROTOCOL_FRAME_ID&(0xFFFFFF)) == ((message.can_id)&(0xFFFFFF)))
        {
            u8Status|=PROTOCOL_MASK;
        }
        _FramesQueue.pop();
    }
    return u8Status;
}

bool TTComm::bWaritForTTControlReadiness()
{
    can_frame message;
    while(_CanHandler.bReceiveFrame(&message))
    {
        if((PROTOCOL_FRAME_ID&(0xFFFFFF)) == ((message.can_id)&(0xFFFFFF)))
        {
            if(message.data[1]==0x01 && message.data[0] == 0x20)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }
    return false;
}


void TTComm::vConfirmAdvantechReadiness()
{
    //Set sending cycle to 1s:
    can_frame message;
    message.can_id = PROTOCOL_RESPONSE_FRAME_ID;
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



void TTComm::vSendVelocities(float fFrontLinear, float fFrontAngular, float fBackLinear, float fBackAngular)
{
    //Set sending cycle to 1s:
    fFrontLinear*=100;
    fFrontAngular*=100000;
    fBackLinear*=100;
    fBackAngular*=100000;
    can_frame message;
    message.can_id = VELOCITY_CONTROL_FRAME_ID;
    message.can_dlc = 8;
    message.data[0] = (uint8_t)(((uint16_t)fFrontLinear)>>8);
    message.data[1] = (uint8_t)fFrontLinear;
    message.data[2] = (uint8_t)(((uint16_t)fFrontAngular)>>8);
    message.data[3] = (uint8_t)fFrontAngular;
    message.data[4] = (uint8_t)(((uint16_t)fBackLinear)>>8);
    message.data[5] = (uint8_t)fBackLinear;;
    message.data[6] = (uint8_t)(((uint16_t)fBackAngular)>>8);
    message.data[7] = (uint8_t)fBackAngular;;
    _CanHandler.iSendFrame(&message);
}

CartsStatus TTComm::GetCartsStatus()
{
    return _Cart;
}

float TTComm::fGetFrontCartAngle()
{
    return _fFrontCartAngle;
}

float TTComm::fGetBackCartAngle()
{
    return _fBackCartAngle;
}
