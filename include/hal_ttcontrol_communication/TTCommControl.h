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
  TtcOk = 0x01,
  TtcAutomaticModeRequest = 0x20,
  TtcCartsEncodersInitialization = 0x30,
  TtcStartStopEngine = 0x40
};

enum ProtocolFrameBytePositions : std::uint8_t
{
  FrameTypeByte = 0x00,
  TtcontrolIDByte = 0x01,
  RequestTtcontrolStateByte = 0x02,
};

enum TtcontrolAutomaticModeRequestByteValues : std::uint8_t
{
  EnterAutomaticSteering = 0x01,
  EnterSafeStateRequest = 0x00
};

enum ReceivedFramesRegisterMask : std::uint32_t
{
  Velocities = (0x00000001 << 0),
  FrontAngle = (0x00000001 << 1),
  BackAngle = (0x00000001 << 2),
  AutomaticStateRequest = (0x00000001 << 4),
  AutomaticStateRequestError = (0x00000001 << 5),
  GeometryFeedback = (0x00000001 << 6),
  TrackEncoderTimeDiff = (0x00000001 << 7),
  TtcontrolStates = (0x00000001 << 8),
  TrackEncoderSpeed = (0x00000001 << 9),
  TrackControl = (0x00000001 << 10),
  TrackEncoderSpeedDesired = (0x00000001 << 11)
};

enum SignalsParametersBytePosition : std::uint8_t
{
  DitterPeriodByte = 1,
  DitterAmplitudeFirstByte = 2,
  EncoderPeriodByte = 4
};

enum CaterpillarPidParametersBytePosition : std::uint8_t
{
  KpCaterpillarFirstByte = 1,
  KiCaterpillarFirstByte = 3,
  KdCaterpillarFirstByte = 5
};

enum CartsPidParametersBytePositions : std::uint8_t
{
  KpFirstByte = 1,
  KiFirstByte = 3,
  KdFirstByte = 5,
  AccelerationFirstByte = 6
};

enum SignalBaseBytePositions : std::uint8_t
{
  ForwardSignalBaseFirstByte = 1,
  BackwardSignalBaseFirstByte = 3
};

enum OverrideAndValvesLimitsBytePositions : std::uint8_t
{
  CartRotationLimitByte = 1,
  OverrideLinearVelocityByte = 2,
  OverrideAngularVelocityByte = 3,
  OverrideValvesLimitsByte = 4,
  DeltasGeometryValvesPushLimitFrontByte = 5,
  DeltasGeometryValvesPushLimitBackByte = 6,
  DeltasGeometryValvesPullFrontBackByte = 7
};

enum StabilizationBytePositions : std::uint8_t
{
  StabilizationKpByte = 1,
  StabilizationKdByte = 2,
  StabilizationKiByte = 3,
  StabilizationMaximumSignalByte = 4
};

enum OilAndEngineBytePositions : std::uint8_t
{
  MaxOilTemperatureByte = 1,
  MinOilTemperatureByte = 2,
  HisteresisOilTemperatureByte = 3,
  RpmMaxByte = 4,
  RpmMinByte = 6
};

enum TransmitCanIds : std::uint32_t
{
  ProtocolResponseFrame = 0x00EF64C8,
  DeltasGeometryCommandFrame = 0x00FFDCC8,
  VelocityControlFrame = 0x00FFD0C8,
  ConfigFrame = 0x00FFDBC8,
  TplControlFront = 0x00FFD1C8,
  TplControlBack = 0x00FFD2C8
};

enum ReceiveCanIds : std::uint32_t
{
  FrontAngleFrame = 0xAA20,
  BackAngleFrame = 0xAA21,
  VelocityFrame = 0x00FFD064,
  DeltasGeometryFeedbackFrame = 0x00FFDC64,
  EngineFeedbackFrame = 0x00FFD664,
  TtcontrolStateFrame = 0x00FFD764,
  ProtocolFrame = 0x00EFC864,
  TrackEncoderTimeDiffFrame = 0x00FFD364,
  TrackEncoderSpeedFrame = 0x00FFD864,
  TrackEncoderSpeedDesiredFrame = 0x00FFDE64,
  TrackControlFrame = 0x00FFDD64
};

enum TTControlParameterType : std::uint8_t
{
  Signals = 0,
  FrontPidT = 1,
  FrontPidR = 2,
  BackPidT = 3,
  BackPidR = 4,
  FrontLeftCaterpillarPid = 5,
  FrontLeftCaterpillarSignalBase = 6,
  FrontRightCaterpillarPid = 7,
  FrontRightCaterpillarSignalBase = 8,
  BackLeftCaterpillarPid = 9,
  BackLeftCaterpillarSignalBase = 10,
  BackRightCaterpillarPid = 11,
  BackRightCaterpillarSignalBase = 12,
  OverrideSpeedAndValveLimits = 13,
  Stabilization = 14,
  OilAndEngine = 15
};

enum VelocityType
{
  FrontLinear = 0,
  FrontAngular = 2,
  BackLinear = 4,
  BackAngular = 6
};

enum TTControlEngineCommands : std::uint8_t
{
  StopEngine = 0,
  StartEngine = 1
};

enum class SelectedTpl
{
  Front,
  Back
};

enum TtcontrolState : std::uint8_t
{
  Initialization = 0,
  WaitingForHandshake = 1,
  CommunicationActive = 2,
  SteeringActive = 3
};

struct TtcontrolCurrentStates
{
  TtcontrolState automatic_state;
  TtcontrolState manual_state;
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
  std::uint8_t err_code_ = ProtocolFrameBytesValues::TtcOk;
  std::string path_to_encoder_file_;
  std::string file_name_ = "/carts_encoder_state_files/carts_encoder_state_data.txt";
  std::string file_name_backup_ = "/carts_encoder_state_files/carts_encoder_state_data_backup.txt";
  std::ifstream encoder_data_file_input_;
  std::ofstream encoder_data_file_output_;
  TtcontrolCurrentStates ttcontrol_current_state_;
  bool ttcontrol_is_connected_;
  bool ttcontrol_carts_angle_confirmed_;
  bool ttcontrol_engine_is_ready_;
  void prepareVelocitiesFrame(can_frame& frame, const float velocity,  // NOLINT(runtime/references)
                              const std::uint8_t start_byte) const;
  float decodeVelocitiy(const std::uint8_t start_byte, const can_frame& frame) const;
  float decodeAngle(const can_frame& frame) const;
  std::uint32_t decodeEncoderValue(const can_frame& frame) const;
  void decodeTtcontrolState(const can_frame& frame);
  void decodeCartsGeometryState(const can_frame& frame);
  void decodeTrackEncoderTimeDiffValue(const can_frame& frame);
  void decodeTrackEncoderSpeedValue(const can_frame& frame);
  double decodeSpeedDesired(const uint8_t start_byte, const can_frame& frame) const;
  void decodeTrackEncoderSpeedDesiredValue(const can_frame& frame);
  void decodeTrackControl(const can_frame& frame);
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
  inline std::int32_t checkTrackEncoderSumSignBit(std::uint8_t byte_to_check, std::uint8_t bit_mask) const;

public:
  TTCommControl();
  ~TTCommControl();
  void initializeCanHandler(std::unique_ptr<ICan> can_handler);
  void initializeCanHandler(const std::string& selected_can);
  void update();
  void receiveCanFrames();
  void parseCanFrames();
  bool getTtcontrolConnectionStatus() const;
  bool getEncodersConfirmedStatus() const;
  bool getEngineStatus() const;
  void requestChangeOfControlState(TtcontrolAutomaticModeRequestByteValues request);
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
  TtcontrolCurrentStates getTtcontrolCurrentState() const;
  std::int32_t getErrorCode() const;
  void clearReceivedCanBusIdBit(const std::uint32_t mask_to_check);
  bool checkReceivedCanBusIdBit(const std::uint32_t mask_to_check) const;
  void prepareTplControlFrame(const SelectedTpl selection, const TplControl& control_values,
                              can_frame& message) const;  // NOLINT(runtime/references)
  void sendTplValveControlViaCan(const SelectedTpl selection, const TplControl& control_values) const;
};
