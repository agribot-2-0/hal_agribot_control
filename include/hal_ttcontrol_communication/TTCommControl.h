/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_common/Can.h>
#include <agribot_common/CanBus/ICan.h>
#include <agribot_msgs/ChangeDeltasGeometry.h>
#include <hal_ttcontrol_communication/CartsStatus.h>
#include <hal_ttcontrol_communication/TplControl.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <queue>
#include <string>

struct TtcontrolOverrideParameters
{
  std::uint8_t cart_rotation_limit;
  double override_velocity_linear;
  double override_velocity_angular;
  std::uint8_t override_valves_limits;
  std::uint8_t deltas_geometry_valves_push_limit_front;
  std::uint8_t deltas_geometry_valves_push_limit_back;
  std::uint8_t deltas_geometry_valves_pull_front_back;
};

enum ProtocolFrameBytesValues : std::uint8_t
{
  ttc_ok = 0x01,
  ttc_automatic_mode_request = 0x20,
  ttc_carts_encoders_initialization = 0x30,
  ttc_start_stop_engine = 0x40
};

enum ReceivedFramesRegisterMask : std::uint32_t
{
  velocities = 0x01,
  front_angle = 0x02,
  back_angle = 0x04,
  automatic_state_request = 0x10,
  automatic_state_request_error = 0x20,
  geometry_feedback = 0x40,
  track_encoder_sum = 0x80
};

enum SignalsParametersBytePosition : std::uint8_t
{
  ditter_period_byte = 1,
  ditter_amplitude_first_byte = 2,
  encoder_period_byte = 4
};

enum CaterpillarPidParametersBytePosition : std::uint8_t
{
  kp_caterpillar_first_byte = 1,
  ki_caterpillar_first_byte = 3,
  kd_caterpillar_first_byte = 5
};

enum CartsPidParametersBytePositions : std::uint8_t
{
  kp_first_byte = 1,
  ki_first_byte = 3,
  kd_first_byte = 5,
  acceleration_first_byte = 6
};

enum SignalBaseBytePositions : std::uint8_t
{
  forward_signal_base_first_byte = 1,
  backward_signal_base_first_byte = 3
};

enum OverrideAndValvesLimitsBytePositions : std::uint8_t
{
  cart_rotation_limit_byte = 1,
  override_linear_velocity_byte = 2,
  override_angular_velocity_byte = 3,
  override_valves_limits_byte = 4,
  deltas_geometry_valves_push_limit_front_byte = 5,
  deltas_geometry_valves_push_limit_back_byte = 6,
  deltas_geometry_valves_pull_front_back_byte = 7
};

enum StabilizationBytePositions : std::uint8_t
{
  stabilization_kp_byte = 1,
  stabilization_kd_byte = 2,
  stabilization_ki_byte = 3,
  stabilization_maximum_signal_byte = 4
};

enum OilAndEngineBytePositions : std::uint8_t
{
  max_oil_temperature_byte = 1,
  min_oil_temperature_byte = 2,
  histeresis_oil_temperature_byte = 3,
  rpm_max_byte = 4,
  rpm_min_byte = 6
};

enum TransmitCanIds : std::uint32_t
{
  protocol_response_frame = 0x00EF64C8,
  deltas_geometry_command_frame = 0x00FFDCC8,
  engine_command_frame = 0x00FFD4C8,
  velocity_control_frame = 0x00FFD0C8,
  config_frame = 0x00FFDBC8,
  tpl_control_front = 0x00FFD1C8,
  tpl_control_back = 0x00FFD2C8
};

enum ReceiveCanIds : std::uint32_t
{
  front_angle_frame = 0xAA20,
  back_angle_frame = 0xAA21,
  velocity_frame = 0x00FFD064,
  deltas_geometry_feedback_frame = 0x00FFDC64,
  engine_feedback_frame = 0x00FFD664,
  protocol_frame = 0x00EFC864,
  track_encoder_sum_frame = 0x00FFD364
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
  back_right_caterpillar_signal_base = 12,
  override_speed_and_valve_limits = 13,
  stabilization = 14,
  oil_and_engine = 15
};

enum VelocityType
{
  front_linear = 0,
  front_angular = 2,
  back_linear = 4,
  back_angular = 6
};

enum TTControlEngineCommands : std::uint8_t
{
  stop_engine = 0,
  start_engine = 1
};

enum class SelectedTpl
{
  front,
  back
};

class TTCommControl
{
  std::unique_ptr<ICan> can_handler_;
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
  std::uint8_t err_code_ = ProtocolFrameBytesValues::ttc_ok;
  std::string path_to_encoder_file_;
  std::string file_name_ = "/carts_encoder_state_files/carts_encoder_state_data.txt";
  std::string file_name_backup_ = "/carts_encoder_state_files/carts_encoder_state_data_backup.txt";
  std::ifstream encoder_data_file_input_;
  std::ofstream encoder_data_file_output_;
  bool ttcontrol_is_connected_;
  bool ttcontrol_carts_angle_confirmed_;
  bool ttcontrol_engine_is_ready_;
  void prepareVelocitiesFrame(can_frame& frame, const float velocity,  // NOLINT(runtime/references)
                              const std::uint8_t start_byte) const;
  float decodeVelocitiy(const std::uint8_t start_byte, const can_frame& frame) const;
  float decodeAngle(const can_frame& frame) const;
  std::uint32_t decodeEncoderValue(const can_frame& frame) const;
  void decodeCartsGeometryState(const can_frame& frame);
  void decodeTrackEncoderSumValue(const can_frame& frame);
  void decodeEngineFeedback(const can_frame& frame);
  void setVelocity(const VelocityType velocity_specification, const can_frame& frame);
  inline bool checkIfCanBusIdsMatch(const std::uint32_t id_to_check, const std::uint32_t msg_id) const;
  inline void setReceivedCanBusIdsBitMask(const std::uint32_t mask_to_set);
  inline bool checkIfBitIsSet(const std::uint32_t number_to_check, const std::uint32_t mask_to_check) const;
  inline void clearBitInReceivedMsgsRegister(const std::uint32_t mask_to_check);
  inline std::uint8_t returnDeltasGeometryCommandByte(const std::uint8_t command) const;
  inline std::uint8_t extractFourthByte(const std::uint32_t four_byte_number) const;
  inline std::uint8_t extractThirdByte(const std::uint32_t four_byte_number) const;
  inline std::uint8_t extractSecondByte(const std::uint32_t four_byte_number) const;
  inline std::uint8_t extractFirstByte(const std::uint32_t four_byte_number) const;
  inline std::uint8_t getLowerHalfOfByte(const std::uint8_t byte) const;
  inline std::uint8_t getUpperHalfOfByte(const std::uint8_t byte) const;
  inline std::uint32_t parseBytesToU32Lsb(uint8_t start_byte, const can_frame& frame) const;
  inline std::int32_t checkTrackEncoderSumSignBit(std::uint8_t bit_mask, std::uint8_t byte_to_check) const;

public:
  TTCommControl();
  ~TTCommControl();
  void initializeCanHandler(std::unique_ptr<ICan> can_handler);
  void initializeCanHandler(std::string selected_can);
  void update();
  void receiveCanFrames();
  void parseCanFrames();
  void setTtcontrolConnectionStatus(const bool& connected);
  bool getTtcontrolConnectionStatus() const;
  bool getEncodersConfirmedStatus() const;
  bool getEngineStatus() const;
  void confirmAdvantechReadiness();
  void setPathToCartsEncodersDataFile(const std::string& path_to_file);
  bool readCartsEncodersDataFile(const bool backup_file);
  std::uint32_t extractEncoderInitValueFromFile(std::ifstream& file_input) const;
  bool writeEncodersDataFile();
  void sendDeltasGeometryCommand(const std::uint8_t front_cart_command, const std::uint8_t back_cart_command);
  void sendCartsEncodersInitialization();
  void sendEngineCommand(const TTControlEngineCommands command);
  void sendVelocities(const float front_linear, const float front_angular, const float back_linear,
                      const float back_angular);
  void sendSignalsParameters(const int ditter_period, const int ditter_amplitude, const int encoder_period);
  void sendCartsPidParameters(const double kp, const double ki, const double kd, const double acceleration,
                              const std::uint8_t frame_type);
  void sendCaterpillarPidParameters(const std::uint32_t kp, const std::uint32_t ki, const std::uint32_t kd,
                                    const std::uint8_t frame_type);
  void sendCaterpillarBaseSignal(const std::uint32_t forward_base, const std::uint32_t backward_base,
                                 const std::uint8_t frame_type);
  void sendOverrideLimits(const TtcontrolOverrideParameters& params);
  void sendStabilizationParameters(const double stab_kp, const double stab_kd, const double stab_ki,
                                   const std::uint8_t stab_max_signal);
  void sendOilAndEngineParams(const std::uint8_t max_oil_temperature, const std::uint8_t min_oil_temperature,
                              const std::uint8_t histeresis_oil_temperature, const std::uint16_t rpm_max,
                              const std::uint16_t rpm_min);
  const CartsStatus& getCartsStatus() const;
  float getFrontCartAngle() const;
  float getBackCartAngle() const;
  std::int32_t getErrorCode() const;
  void clearReceivedCanBusIdBit(const std::uint32_t mask_to_check);
  bool checkReceivedCanBusIdBit(const std::uint32_t mask_to_check) const;
  void prepareTplControlFrame(const SelectedTpl selection, const TplControl& control_values,
                              can_frame& message) const;  // NOLINT(runtime/references)
  void sendTplValveControlViaCan(const SelectedTpl selection, const TplControl& control_values) const;
};
