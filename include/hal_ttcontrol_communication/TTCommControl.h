/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <math.h>
#include <agribot_common/Can.h>
#include <hal_ttcontrol_communication/CartsStatus.h>
#include <iostream>
#include <fstream>
#include <queue>
#include <string>

constexpr uint8_t VELOCITIES_MASK = 0x01;
constexpr uint8_t FRONT_ANGLE_MASK = 0x02;
constexpr uint8_t BACK_ANGLE_MASK = 0x04;
constexpr uint8_t AUTOMATIC_STATE_REQUEST_MASK = 0x10;
constexpr uint8_t AUTOMATIC_STATE_REQUEST_ERROR_MASK = 0x20;
constexpr uint8_t TTC_OK = 0x01;
constexpr uint8_t TTC_AUTOMATIC_MODE_REQUEST = 0x20;
constexpr uint8_t TTC_CARTS_ANGLE_INITIALIZATION = 0x30;

constexpr uint8_t CONFIG_FRAME_TYPE_BYTE = 0;
constexpr uint8_t DITTER_PERIOD_BYTE = 1;
constexpr uint8_t DITTER_AMPLITUDE_FIRST_BYTE = 2;
constexpr uint8_t ENCODER_PERIOD_BYTE = 4;
constexpr uint8_t KP_FIRST_BYTE = 1;
constexpr uint8_t KI_FIRST_BYTE = 3;
constexpr uint8_t KD_FIRST_BYTE = 5;
constexpr uint8_t ACC_FIRST_BYTE = 6;
constexpr uint K_RESOLUTION_MULTIPLIER = 10000;
constexpr uint KD_RESOLUTION_MULTIPLIER = 100;
constexpr uint ACC_RESOLUTION_MULTIPLIER = 1000;
constexpr uint8_t KP_CATERPILLAR_FIRST_BYTE = 1;
constexpr uint8_t KI_CATERPILLAR_FIRST_BYTE = 3;
constexpr uint8_t KD_CATERPILLAR_FIRST_BYTE = 5;
constexpr uint8_t FORWARD_SIGNAL_BASE_FIRST_BYTE = 1;
constexpr uint8_t BACKWARD_SIGNAL_BASE_FIRST_BYTE = 3;

enum eVelocities
{
  FRONT_LINEAR = 0,
  FRONT_ANGULAR = 2,
  BACK_LINEAR = 4,
  BACK_ANGULAR = 6
};

class TTCommControl
{
  Can _CanHandler;
  std::queue<can_frame> _FramesQueue;
  CartsStatus _Cart;
  float _fFrontCartAngle;
  bool _bFrontCartAngleFresh;
  float _fBackCartAngle;
  bool _bBackCartAngleFresh;
  float _fInitFrontCartAngle;
  float _fInitBackCartAngle;
  uint8_t _iErrCode = TTC_OK;
  std::string _path_to_angle_file;
  std::string _file_name = "/angle_files/angle_data.txt";
  std::string _file_name_backup = "/angle_files/angle_data_backup.txt";
  void vPrepareVelocitiesFrame(can_frame* Frame, float f16Velocity, uint8_t u8StartBit);
  float fDecodeVelocitiy(uint8_t u8StartBit, can_frame Frame);
  float fDecodeAngle(can_frame Frame);
  void vSetVelocitiy(eVelocities velo, can_frame Frame);
  std::ifstream _angle_data_file_input;
  std::ofstream _angle_data_file_output;
  bool _ttcontrol_is_connected;
  bool _ttcontrol_carts_angle_confirmed;

public:
  const struct eTTControlParamsType
  {
    uint8_t TTCONTROL_SIGNALS_PARAMS_TYPE = 0;
    uint8_t TTCONTROL_FRONT_PID_T_PARAMS_TYPE = 1;
    uint8_t TTCONTROL_FRONT_PID_R_PARAMS_TYPE = 2;
    uint8_t TTCONTROL_BACK_PID_T_PARAMS_TYPE = 3;
    uint8_t TTCONTROL_BACK_PID_R_PARAMS_TYPE = 4;
    uint8_t TTCONTROL_FRONT_LEFT_CATERPILLAR_PID_PARAMS_TYPE = 5;
    uint8_t TTCONTROL_FRONT_LEFT_CATERPILLAR_SIGNAL_BASE_TYPE = 6;
    uint8_t TTCONTROL_FRONT_RIGHT_CATERPILLAR_PID_PARAMS_TYPE = 7;
    uint8_t TTCONTROL_FRONT_RIGHT_CATERPILLAR_SIGNAL_BASE_TYPE = 8;
    uint8_t TTCONTROL_BACK_LEFT_CATERPILLAR_PID_PARAMS_TYPE = 9;
    uint8_t TTCONTROL_BACK_LEFT_CATERPILLAR_SIGNAL_BASE_TYPE = 10;
    uint8_t TTCONTROL_BACK_RIGHT_CATERPILLAR_PID_PARAMS_TYPE = 11;
    uint8_t TTCONTROL_BACK_RIGHT_CATERPILLAR_SIGNAL_BASE_TYPE = 12;
  } TTControlParamsFrameType;
  explicit TTCommControl(const char* pchCanChannel);
  ~TTCommControl();
  void update();
  void vReceiveCanFrames();
  uint8_t u8ParseCanFrames();
  void vSetTTControlConnectionStatus(bool connected);
  bool bGetTTControlConnectionStatus();
  bool bGetAnglesConfirmedStatus();
  void vConfirmAdvantechReadiness();
  bool bWaritForTTControlReadiness();
  void vSetPathToAngleDataFile(std::string path_to_file);
  bool bReadAngleDataFile(bool backup_file);
  bool bWriteAngleDataFile();
  void vSendCartsAnglesInitialization();
  void vSendVelocities(float fFrontLinear, float fFrontAngular, float fBackLinear, float fBackAngular);
  void vSendSignalsParameters(int ditter_period, int ditter_amplitude, int encoder_period);
  void vSendCartsPIDParameters(double kp, double ki, double kd, double acc, uint8_t frame_type);
  void vSendCaterpillarPIDParameters(uint kp, uint ki, uint kd, uint8_t frame_type);
  void vSendCaterpillarBaseSignal(uint forward_base, uint backward_base, uint8_t frame_type);
  CartsStatus GetCartsStatus();
  float fGetFrontCartAngle();
  float fGetBackCartAngle();
  int iGetErrorCode();
};
