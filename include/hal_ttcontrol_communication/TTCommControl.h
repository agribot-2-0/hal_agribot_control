/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_common/Can.h>
#include <hal_ttcontrol_communication/CartsStatus.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <queue>
#include <string>

constexpr uint8_t VELOCITIES_MASK = 0x01;
constexpr uint8_t FRONT_ANGLE_MASK = 0x02;
constexpr uint8_t BACK_ANGLE_MASK = 0x04;
constexpr uint8_t TRACK_ENCODER_SUM_MASK = 0x08;
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
  Can _can_handler;
  std::queue<can_frame> _frames_queue;
  CartsStatus _cart;
  float _front_cart_angle;
  uint _init_front_cart_encoder_value;
  uint _front_cart_encoder_value;
  bool _front_cart_encoder_unread;
  float _back_cart_angle;
  uint _init_back_cart_encoder_value;
  uint _back_cart_encoder_value;
  bool _back_cart_encoder_unread;
  uint8_t _err_code = TTC_OK;
  std::string _path_to_encoder_file;
  std::string _file_name = "/carts_encoder_state_files/carts_encoder_state_data.txt";
  std::string _file_name_backup = "/carts_encoder_state_files/carts_encoder_state_data_backup.txt";
  void vPrepareVelocitiesFrame(can_frame* frame, float velocity, uint8_t start_byte);
  float fDecodeVelocitiy(uint8_t start_byte, can_frame frame);
  float fDecodeAngle(can_frame frame);
  uint uiDecodeEncoderValue(can_frame frame);
  void vDecodeTrackEncoderSumValue(const can_frame frame);
  void vSetVelocitiy(eVelocities enum_velo, can_frame frame);
  std::ifstream _encoder_data_file_input;
  std::ofstream _encoder_data_file_output;
  bool _ttcontrol_is_connected;
  bool _ttcontrol_carts_angle_confirmed;
  inline uint8_t u8GetLowerHalfOfByte(const uint8_t byte);
  inline uint8_t u8GetUpperHalfOfByte(const uint8_t byte);

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
  void vSetTTControlConnectionStatus(const bool& connected);
  bool bGetTTControlConnectionStatus();
  bool bGetEncodersConfirmedStatus();
  void vConfirmAdvantechReadiness();
  bool bWaritForTTControlReadiness();
  void vSetPathToCartsEncodersDataFile(const std::string& path_to_file);
  bool bReadCartsEncodersDataFile(const bool& backup_file);
  uint iExtractEncoderInitValueFromFile(std::ifstream& file_input);
  bool bWriteEncodersDataFile();
  void vSendCartsEncodersInitialization();
  void vSendVelocities(const float& front_linear, const float& front_angular, const float& back_linear,
                       const float& back_angular);
  void vSendSignalsParameters(const int& ditter_period, const int& ditter_amplitude, const int& encoder_period);
  void vSendCartsPIDParameters(const double& kp, const double& ki, const double& kd, const double& acc,
                               const uint8_t& frame_type);
  void vSendCaterpillarPIDParameters(const uint& kp, const uint& ki, const uint& kd, const uint8_t& frame_type);
  void vSendCaterpillarBaseSignal(const uint& forward_base, const uint& backward_base, const uint8_t& frame_type);
  CartsStatus getCartsStatus();
  float fGetFrontCartAngle();
  float fGetBackCartAngle();
  int iGetErrorCode();
};
