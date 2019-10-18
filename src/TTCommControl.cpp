#include<TTCommControl.h>

#define PROTOCOL_FRAME_ID 0x000EFC864
#define PROTOCOL_RESPONSE_FRAME_ID 0x00EF64C8
#define VELO_FRAME_ID 0x00FFD064
#define FRONT_ANGLE_ID 0xAA20
#define BACK_ANGLE_ID 0xAA21
#define VELOCITY_CONTROL_FRAME_ID 0x00FFD0C8

TTCommControl::TTCommControl()
{
    _CanHandler.iInitialize("can0");
}

void TTCommControl::vReceiveCanFrames()
{
    can_frame message;
    while(_CanHandler.bReceiveFrame(&message))
    {
        _FramesQueue.push(message);
    }
}

uint8_t TTCommControl::u8ParseCanFrames()
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
            if(message.data[2]&0x80)
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
            _fFrontCartAngle = ((float)(((int32_t)message.data[2])<<16 | ((int32_t)message.data[1])<<8
            | ((int32_t)message.data[0])))*0.214-180;
            u8Status|=FRONT_ANGLE_MASK;
        }
        if((BACK_ANGLE_ID&(0xFFFF)) == ((message.can_id)&(0xFFFF)))
        {
            _fBackCartAngle = ((float)(((int32_t)message.data[2])<<16 | ((int32_t)message.data[1])<<8
            | ((int32_t)message.data[0])))*0.214-180;
            u8Status|=BACK_ANGLE_MASK;
        }
        if((PROTOCOL_FRAME_ID&(0xFFFFFF)) == ((message.can_id)&(0xFFFFFF)))
        {
            if(message.data[0] == TTC_AUTOMATIC_MODE_REQUEST)
            {
                if(message.data[1]==TTC_OK)
                {
                    u8Status|=AUTOMATIC_STATE_REQUEST_MASK;
                }
                else
                {
                    _iErrCode = message.data[1];
                    u8Status|=AUTOMATIC_STATE_REQUEST_ERROR_MASK;
                }
            }
        }
        _FramesQueue.pop();
    }
    return u8Status;
}

void TTCommControl::vConfirmAdvantechReadiness()
{
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

void TTCommControl::vSendVelocities(float fFrontLinear, float fFrontAngular, float fBackLinear, float fBackAngular)
{
    //Set sending cycle to 1s:
    int16_t i16FrontLinear=(100*fFrontLinear);
    int16_t i16FrontAngular=(100*fFrontAngular);
    int16_t i16BackLinear=(100*fBackLinear);
    int16_t i16BackAngular=(100*fBackAngular);

    can_frame message;
    message.can_id = VELOCITY_CONTROL_FRAME_ID;
    message.can_dlc = 8;
    message.data[0] = (uint8_t)((i16FrontLinear&0xFF00)>>8);
    if(i16FrontLinear<0)
    {
        i16FrontLinear=32768-i16FrontLinear;
        message.data[0] = (uint8_t)((i16FrontLinear&0xFF00)>>8);
        message.data[0]|=0x80;
    }
    message.data[1] = (uint8_t)i16FrontLinear;

    message.data[2] = (uint8_t)((i16FrontAngular)>>8);
    if(i16FrontAngular<0)
    {
        i16FrontAngular=32768-i16FrontAngular;
        message.data[2] = (uint8_t)((i16FrontAngular&0xFF00)>>8);
        message.data[2] |= 0x80;
    }
    message.data[3] = (uint8_t)i16FrontAngular;

    message.data[4] = (uint8_t)((i16BackLinear&0xFF00)>>8);
    if(i16BackLinear<0)
    {
        i16BackLinear = 32768-i16BackLinear;
        message.data[4] |= (uint8_t)((i16BackLinear&0xFF00)>>8);
        message.data[4] = 0x80;
    }
    message.data[5] = (uint8_t)i16BackLinear;;

    message.data[6] = (uint8_t)((i16BackAngular&0xFF00)>>8);
    if(i16BackAngular<0)
    {
        i16BackAngular = 32768-i16BackAngular;
        message.data[6] |= (uint8_t)((i16BackAngular&0xFF00)>>8);
        message.data[6] = 0x80;
    }
    message.data[7] = (uint8_t)i16BackAngular;;
    _CanHandler.iSendFrame(&message);
}

CartsStatus TTCommControl::GetCartsStatus()
{
    return _Cart;
}

float TTCommControl::fGetFrontCartAngle()
{
    return _fFrontCartAngle;
}

float TTCommControl::fGetBackCartAngle()
{
    return _fBackCartAngle;
}

int TTCommControl::iGetErrorCode()
{
    return _iErrCode;
}