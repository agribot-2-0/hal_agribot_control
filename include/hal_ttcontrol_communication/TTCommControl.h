#pragma once

#include <agribot_common/Can.h>
#include <hal_ttcontrol_communication/CartsStatus.h>
#include <queue>

#define VELOCITIES_MASK 0x01
#define FRONT_ANGLE_MASK 0x02
#define BACK_ANGLE_MASK 0x04
#define AUTOMATIC_STATE_REQUEST_MASK 0x10
#define AUTOMATIC_STATE_REQUEST_ERROR_MASK 0x20

#define TTC_OK 0x01
#define TTC_AUTOMATIC_MODE_REQUEST 0x20

enum eVelocities {FRONT_LINEAR, FRONT_ANGULAR, BACK_LINEAR, BACK_ANGULAR};

class TTCommControl
{
    Can _CanHandler;
    std::queue<can_frame> _FramesQueue;
    CartsStatus _Cart;
    float _fFrontCartAngle;
    float _fBackCartAngle;
    uint8_t _iErrCode = TTC_OK;
    void vPrepareVelocitiesFrame(can_frame* Frame, float f16Velocity, uint8_t u8StartBit);
    void vSetVelocities(eVelocities velo, can_frame Frame);
public:
    TTCommControl(char* pchCanChannel);
    ~TTCommControl();
    void vReceiveCanFrames();
    uint8_t u8ParseCanFrames();
    void vConfirmAdvantechReadiness();
    bool bWaritForTTControlReadiness();
    void vSendVelocities(float fFrontLinear, float fFrontAngular, float fBackLinear, float fBackAngular);
    CartsStatus GetCartsStatus();
    float fGetFrontCartAngle();
    float fGetBackCartAngle();
    int iGetErrorCode();
};