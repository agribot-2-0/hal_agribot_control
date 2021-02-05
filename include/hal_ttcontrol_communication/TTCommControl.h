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

constexpr std::uint8_t VELOCITIES_MASK = 0x01;
constexpr std::uint8_t FRONT_ANGLE_MASK = 0x02;
constexpr std::uint8_t BACK_ANGLE_MASK = 0x04;
constexpr std::uint8_t AUTOMATIC_STATE_REQUEST_MASK = 0x10;
constexpr std::uint8_t AUTOMATIC_STATE_REQUEST_ERROR_MASK = 0x20;
constexpr std::uint8_t GEOMETRY_FEEDBACK_MASK = 0x40;
constexpr std::uint8_t TTC_OK = 0x01;
constexpr std::uint8_t TTC_AUTOMATIC_MODE_REQUEST = 0x20;
constexpr std::uint8_t TTC_CARTS_ANGLE_INITIALIZATION = 0x30;

enum ConfigFrameBytePositions : uint8_t
{
  config_frame_type_byte = 0,
  ditter_period_byte = 1,
  ditter_amplitude_first_byte = 2,
  encoder_period_byte = 4,
  kp_first_byte = 1,
  ki_first_byte = 3,
  kd_first_byte = 5,
  kp_caterpillar_first_byte = 1,
  ki_caterpillar_first_byte = 3,
  kd_caterpillar_first_byte = 5,
  forward_signal_base_first_byte = 1,
  backward_signal_base_first_byte = 3,
  acceleration_first_byte = 6
};

enum TransmitCanIds : uint32_t
{
  protocol_response_frame = 0x00EF64C8,
  deltas_geometry_command_frame = 0x00FFDCC8,
  velocity_control_frame = 0x00FFD0C8,
  config_frame = 0x00FFDBC8
};

enum ReceiveCanIds : uint32_t
{
  front_angle_frame = 0xAA20,
  back_angle_frame = 0xAA21,
  velocity_frame = 0x00FFD064,
  deltas_geometry_feedback_frame = 0x00FFDC64,
  protocol_frame = 0x00EFC864
};

enum TTControlParameterType : std::uint8_t
{
  signals = 0,
  front_pid_t = 1,
  front_pid_r = 2,
  back_pid_t = 3,
  back_pid_r = 4,
  front_left_caterpillar_pid = 5,
  front_left_caterpillar_signal_base = 6,
  front_right_caterpillar_pid = 7,
  front_right_caterpillar_signal_base = 8,
  back_left_caterpillar_pid = 9,
  back_left_caterpillar_signal_base = 10,
  back_right_caterpillar_pid = 11,
  back_right_caterpillar_signal_base = 12
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
  std::uint32_t received_msgs_register_;
  float front_cart_angle_;
  std::uint32_t init_front_cart_encoder_value_;
  std::uint32_t front_cart_encoder_value_;
  bool front_cart_encoder_unread_;
  float back_cart_angle_;
  std::uint32_t init_back_cart_encoder_value_;
  std::uint32_t back_cart_encoder_value_;
  bool back_cart_encoder_unread_;
  std::uint8_t err_code_ = TTC_OK;
  std::string path_to_encoder_file_;
  std::string file_name_ = "/carts_encoder_state_files/carts_encoder_state_data.txt";
  std::string file_name_backup_ = "/carts_encoder_state_files/carts_encoder_state_data_backup.txt";
  std::ifstream encoder_data_file_input_;
  std::ofstream encoder_data_file_output_;
  bool ttcontrol_is_connected_;
  bool ttcontrol_carts_angle_confirmed_;
  void prepareVelocitiesFrame(can_frame& frame, const float velocity,  // NOLINT(runtime/references)
                              const std::uint8_t start_byte);
  float decodeVelocitiy(const std::uint8_t start_byte, const can_frame& frame);
  float decodeAngle(const can_frame& frame);
  std::uint32_t decodeEncoderValue(const can_frame& frame);
  void decodeCartsGeometryState(const can_frame& frame);
  void setVelocity(const VelocityType velocity_specification, const can_frame& frame);
  inline bool checkIfCanBusIdsMatch(const std::uint32_t id_to_check, const std::uint32_t msg_id);
  inline void setReceivedCanBusIdsBitMask(const std::uint32_t mask_to_set);
  inline bool checkIfBitIsSet(const std::uint32_t number_to_check, const std::uint32_t mask_to_check);
  inline void clearBitInReceivedMsgsRegister(const std::uint32_t mask_to_check);
  inline std::uint8_t returnDeltasGeometryCommandByte(const std::uint8_t command);
  inline std::uint8_t extractFourthByte(const std::uint32_t four_byte_number);
  inline std::uint8_t extractThirdByte(const std::uint32_t four_byte_number);
  inline std::uint8_t extractSecondByte(const std::uint32_t four_byte_number);
  inline std::uint8_t extractFirstByte(const std::uint32_t four_byte_number);

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
  std::uint32_t extractEncoderInitValueFromFile(std::ifstream& file_input);
  bool writeEncodersDataFile();
  void sendDeltasGeometryCommand(const std::uint8_t front_cart_command, const std::uint8_t back_cart_command);
  void sendCartsEncodersInitialization();
  void sendVelocities(const float front_linear, const float front_angular, const float back_linear,
                      const float back_angular);
  void sendSignalsParameters(const int ditter_period, const int ditter_amplitude, const int encoder_period);
  void sendCartsPidParameters(const double kp, const double ki, const double kd, const double acceleration,
                              const std::uint8_t frame_type);
  void sendCaterpillarPidParameters(const std::uint32_t kp, const std::uint32_t ki, const std::uint32_t kd,
                                    const std::uint8_t frame_type);
  void sendCaterpillarBaseSignal(const std::uint32_t forward_base, const std::uint32_t backward_base,
                                 const std::uint8_t frame_type);
  const CartsStatus& getCartsStatus() const;
  float getFrontCartAngle() const;
  float getBackCartAngle() const;
  std::int32_t getErrorCode() const;
  void clearReceivedCanBusIdBit(const std::uint32_t mask_to_check);
  bool checkReceivedCanBusIdBit(const std::uint32_t mask_to_check);
};
