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

enum eVelocities
{
  FRONT_LINEAR = 0,
  FRONT_ANGULAR = 2,
  BACK_LINEAR = 4,
  BACK_ANGULAR = 6
};

class TTCommControl
{
  static constexpr uint8_t CONFIG_FRAME_TYPE_BYTE = 0;
  static constexpr uint8_t DITTER_PERIOD_BYTE = 1;
  static constexpr uint8_t DITTER_AMPLITUDE_FIRST_BYTE = 2;
  static constexpr uint8_t ENCODER_BYTE = 4;
  static constexpr uint8_t KP_T_FIRST_BYTE = 1;
  static constexpr uint8_t KD_T_FIRST_BYTE = 3;
  static constexpr uint8_t ACC_FIRST_BYTE = 5;

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
  void vSendFrontPIDTranslationParameters(double front_kp_t, double front_kd_t, double front_acc_t);
  void vSendFrontPIDRotationParameters(double front_kp_r, double front_kd_r, double front_acc_r);
  void vSendBackPIDTranslationParameters(double back_kp_t, double back_kd_t, double back_acc_t);
  void vSendBackPIDRotationParameters(double back_kp_r, double back_kd_r, double back_acc_r);
  CartsStatus GetCartsStatus();
  float fGetFrontCartAngle();
  float fGetBackCartAngle();
  int iGetErrorCode();
};
