/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_common/Can.h>
#include <hal_ttcontrol_communication/CartsStatus.h>
#include <math.h>
#include <queue>

constexpr uint8_t VELOCITIES_MASK = 0x01;
constexpr uint8_t FRONT_ANGLE_MASK = 0x02;
constexpr uint8_t BACK_ANGLE_MASK = 0x04;
constexpr uint8_t AUTOMATIC_STATE_REQUEST_MASK = 0x10;
constexpr uint8_t AUTOMATIC_STATE_REQUEST_ERROR_MASK = 0x20;
constexpr uint8_t TTC_OK = 0x01;
constexpr uint8_t TTC_AUTOMATIC_MODE_REQUEST = 0x20;

constexpr uint CONFIG_FRAME_CAN_ID = 0x00FFDBC8;
constexpr uint8_t CONFIG_FRAME_TYPE_BYTE = 0;
constexpr uint8_t DITTER_PERIOD_BYTE = 1;
constexpr uint8_t DITTER_AMPLITUDE_FIRST_BYTE = 2;
constexpr uint8_t ENCODER_PERIOD_BYTE = 4;
constexpr uint8_t KP_FIRST_BYTE = 1;
constexpr uint8_t KD_FIRST_BYTE = 3;
constexpr uint8_t ACC_FIRST_BYTE = 5;
constexpr uint K_RESOLUTION_MULTIPLIER = 10;
constexpr uint ACC_RESOLUTION_MULTIPLIER = 1000;

enum eVelocities
{
  FRONT_LINEAR = 0,
  FRONT_ANGULAR = 2,
  BACK_LINEAR = 4,
  BACK_ANGULAR = 6
};

enum class eTTControlParamsType
{
  TTCONTROL_SIGNALS_PARAMS_TYPE = 0,
  TTCONTROL_FRONT_PID_T_PARAMS_TYPE = 1,
  TTCONTROL_FRONT_PID_R_PARAMS_TYPE = 2,
  TTCONTROL_BACK_PID_T_PARAMS_TYPE = 3,
  TTCONTROL_BACK_PID_R_PARAMS_TYPE = 4
};

class TTCommControl
{
  Can _CanHandler;
  std::queue<can_frame> _FramesQueue;
  CartsStatus _Cart;
  float _fFrontCartAngle;
  float _fBackCartAngle;
  uint8_t _iErrCode = TTC_OK;
  void vPrepareVelocitiesFrame(can_frame* Frame, float f16Velocity, uint8_t u8StartBit);
  float fDecodeVelocitiy(uint8_t u8StartBit, can_frame Frame);
  float fDecodeAngle(can_frame Frame);
  void vSetVelocitiy(eVelocities velo, can_frame Frame);

public:
  explicit TTCommControl(const char* pchCanChannel);
  ~TTCommControl();
  void update();
  void vReceiveCanFrames();
  uint8_t u8ParseCanFrames();
  void vConfirmAdvantechReadiness();
  bool bWaritForTTControlReadiness();
  void vSendVelocities(float fFrontLinear, float fFrontAngular, float fBackLinear, float fBackAngular);
  void vSendSignalsParameters(int ditter_period, int ditter_amplitude, int encoder_period);
  void vSendCartsPIDParameters(double kp, double kd, double acc, eTTControlParamsType frame_type);
  CartsStatus GetCartsStatus();
  float fGetFrontCartAngle();
  float fGetBackCartAngle();
  int iGetErrorCode();
};
