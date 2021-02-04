/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_common/Can.h>
#include <agribot_msgs/ChangeDeltasGeometry.h>
#include <hal_ttcontrol_communication/CartsStatus.h>
#include <cstdint>
#include <cmath>
#include <iostream>
#include <fstream>
#include <queue>
#include <string>

using std::int16_t;
using std::int32_t;
using std::int8_t;
using std::uint16_t;
using std::uint32_t;
using std::uint8_t;

constexpr uint8_t VELOCITIES_MASK = 0x01;
constexpr uint8_t FRONT_ANGLE_MASK = 0x02;
constexpr uint8_t BACK_ANGLE_MASK = 0x04;
constexpr uint8_t AUTOMATIC_STATE_REQUEST_MASK = 0x10;
constexpr uint8_t AUTOMATIC_STATE_REQUEST_ERROR_MASK = 0x20;
constexpr uint8_t GEOMETRY_FEEDBACK_MASK = 0x40;
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
constexpr uint8_t ACCELERATION_FIRST_BYTE = 6;
constexpr uint K_RESOLUTION_MULTIPLIER = 10000;
constexpr uint KD_RESOLUTION_MULTIPLIER = 100;
constexpr uint ACCELERATION_RESOLUTION_MULTIPLIER = 1000;
constexpr uint8_t KP_CATERPILLAR_FIRST_BYTE = 1;
constexpr uint8_t KI_CATERPILLAR_FIRST_BYTE = 3;
constexpr uint8_t KD_CATERPILLAR_FIRST_BYTE = 5;
constexpr uint8_t FORWARD_SIGNAL_BASE_FIRST_BYTE = 1;
constexpr uint8_t BACKWARD_SIGNAL_BASE_FIRST_BYTE = 3;

enum TTControlParamsType : uint8_t
{
  ttcontrol_signals_params_type = 0,
  ttcontrol_front_pid_t_params_type = 1,
  ttcontrol_front_pid_r_params_type = 2,
  ttcontrol_back_pid_t_params_type = 3,
  ttcontrol_back_pid_r_params_type = 4,
  ttcontrol_front_left_caterpillar_pid_params_type = 5,
  ttcontrol_front_left_caterpillar_signal_base_type = 6,
  ttcontrol_front_right_caterpillar_pid_params_type = 7,
  ttcontrol_front_right_caterpillar_signal_base_type = 8,
  ttcontrol_back_left_caterpillar_pid_params_type = 9,
  ttcontrol_back_left_caterpillar_signal_base_type = 10,
  ttcontrol_back_right_caterpillar_pid_params_type = 11,
  ttcontrol_back_right_caterpillar_signal_base_type = 12
};

enum VelocityType
{
  front_linear = 0,
  front_angular = 2,
  back_linear = 4,
  back_angular = 6
};

class TTCommControl
{
  Can can_handler_;
  std::queue<can_frame> frames_queue_;
  CartsStatus cart_;
  uint32_t received_msgs_register_;
  float front_cart_angle_;
  uint32_t init_front_cart_encoder_value_;
  uint32_t front_cart_encoder_value_;
  bool front_cart_encoder_unread_;
  float back_cart_angle_;
  uint32_t init_back_cart_encoder_value_;
  uint32_t back_cart_encoder_value_;
  bool back_cart_encoder_unread_;
  uint8_t err_code_ = TTC_OK;
  std::string path_to_encoder_file_;
  std::string file_name_ = "/carts_encoder_state_files/carts_encoder_state_data.txt";
  std::string file_name_backup_ = "/carts_encoder_state_files/carts_encoder_state_data_backup.txt";
  std::ifstream encoder_data_file_input_;
  std::ofstream encoder_data_file_output_;
  bool ttcontrol_is_connected_;
  bool ttcontrol_carts_angle_confirmed_;
  void prepareVelocitiesFrame(can_frame& frame, const float velocity,  // NOLINT(runtime/references)
                              const uint8_t start_byte);
  float decodeVelocitiy(const uint8_t start_byte, const can_frame& frame);
  float decodeAngle(const can_frame& frame);
  uint32_t decodeEncoderValue(const can_frame& frame);
  void decodeCartsGeometryState(const can_frame& frame);
  void setVelocity(const VelocityType velocity_specification, const can_frame& frame);
  inline bool checkIfCanBusIdsMatch(const uint32_t id_to_check, const uint msg_id);
  inline void setReceivedCanBusIdsBitMask(const uint32_t mask_to_set);
  inline bool checkIfBitIsSet(const uint32_t number_to_check, const uint32_t mask_to_check);
  inline void clearBitInReceivedMsgsRegister(const uint32_t mask_to_check);
  inline uint8_t returnDeltasGeometryCommandByte(const uint8_t command);
  inline uint8_t extractFourthByte(const uint32_t four_byte_number);
  inline uint8_t extractThirdByte(const uint32_t four_byte_number);
  inline uint8_t extractSecondByte(const uint32_t four_byte_number);
  inline uint8_t extractFirstByte(const uint32_t four_byte_number);

public:
  explicit TTCommControl(const char* pchCanChannel);
  ~TTCommControl();
  void update();
  void receiveCanFrames();
  void parseCanFrames();
  void setTtcontrolConnectionStatus(const bool& connected);
  bool getTtcontrolConnectionStatus();
  bool getEncodersConfirmedStatus() const;
  void confirmAdvantechReadiness();
  void setPathToCartsEncodersDataFile(const std::string& path_to_file);
  bool readCartsEncodersDataFile(const bool backup_file);
  uint32_t extractEncoderInitValueFromFile(std::ifstream& file_input);
  bool writeEncodersDataFile();
  void sendDeltasGeometryCommand(const uint8_t front_cart_command, const uint8_t back_cart_command);
  void sendCartsEncodersInitialization();
  void sendVelocities(const float front_linear, const float front_angular, const float back_linear,
                      const float back_angular);
  void sendSignalsParameters(const int ditter_period, const int ditter_amplitude, const int encoder_period);
  void sendCartsPidParameters(const double kp, const double ki, const double kd, const double acceleration,
                              const uint8_t frame_type);
  void sendCaterpillarPidParameters(const uint kp, const uint ki, const uint kd, const uint8_t frame_type);
  void sendCaterpillarBaseSignal(const uint forward_base, const uint backward_base, const uint8_t frame_type);
  const CartsStatus& getCartsStatus() const;
  float getFrontCartAngle() const;
  float getBackCartAngle() const;
  int getErrorCode() const;
  void clearReceivedCanBusIdBit(const uint32_t mask_to_check);
  bool checkReceivedCanBusIdBit(const uint32_t mask_to_check);
};
