/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <hal_ttcontrol_communication/TTCommControl.h>

using std::int16_t;
using std::int32_t;
using std::int8_t;
using std::uint16_t;
using std::uint32_t;
using std::uint8_t;

// node IDs of advantech and TTControl, written here for the reference
constexpr uint8_t TTCONTROL_J1939_NODE_ID = 0x64;
constexpr uint8_t ADVANTECH_J1939_NODE_ID = 0xC8;
// constants for easier bit operations and CAN bus frames parsing
constexpr uint32_t TWO_LAST_BYTES_ADDRESS_MASK = 0xFFFF;
constexpr uint32_t GET_FOURTH_BYTE = 0xFF000000;
constexpr uint32_t GET_THIRD_BYTE = 0x00FF0000;
constexpr uint32_t GET_SECOND_BYTE = 0x0000FF00;
constexpr uint32_t GET_FIRST_BYTE = 0x000000FF;
constexpr uint8_t TPL_CONTROL_FRAME_SIZE = 6;
constexpr uint8_t FULL_CAN_FRAME_BYTE_NUMBER = 8;
constexpr uint8_t TWO_BYTE_CAN_FRAME = 2;
// config type byte position for config parameter frames
constexpr uint8_t config_frame_type_byte = 0;
// protocol data bytes
constexpr uint8_t PROTOCOL_PURPOSE_BYTE = 0;
constexpr uint8_t PROTOCOL_DETAIL_BYTE = 1;
// deltas geometry steering and feedback
constexpr uint8_t FRONT_DELTA_GEOMETRY_BYTE = 0;
constexpr uint8_t BACK_DELTA_GEOMETRY_BYTE = 1;
constexpr bool DELTAS_GEOMETRY_UP = true;
constexpr bool DELTAS_GEOMETRY_DOWN = false;
// Encode/decode velocity frame coming from TTC/to TTC - check TTC-Advantech
// protocol DS for more info
constexpr uint32_t VELOCITY_FRAME_ENCODE_CONST = 1000;
constexpr uint32_t INVERSE_INT16_CONST = 32768;
constexpr uint8_t SIGN_BIT = 0x80;
constexpr uint8_t DATA_BITS = 0x7F;
constexpr float VELOCITY_FRAME_DECODE_CONST = 0.001;
// Decode angle frame - check inclinometer DS for more info
constexpr float ANGLE_FRAME_DECODE_CONST_MULTIPLIER = 0.214;
constexpr float ANGLE_FRAME_DECODE_CONST_OFFSET = -180;
constexpr uint8_t GET_UPPER_HALF_OF_THE_BYTE = 0xF0;
constexpr uint8_t GET_LOWER_HALF_OF_THE_BYTE = 0x0F;
constexpr uint32_t CARTS_ANGLE_RESOLUTION = 1000;
constexpr float RAD_TO_DEG = (180.0 / M_PI);   // TODO(adam): use the function from agribot_common.h
constexpr double DEG_TO_RAD = (M_PI / 180.0);  // TODO(adam): use the function from agribot_common.h
constexpr uint8_t CARTS_ENCODER_HIGH_BYTE_POSITION = 1;
constexpr uint8_t CARTS_ENCODER_LOW_BYTE_POSITION = 0;
// the arbitrary zero of the encoders counter on the TTControl, used for easy
// rounding to the beginning of the measures exceeding the 360 degrees
// the encoders natural limits are from 10000 (-360 deg) to 30000(360 deg)
// but practically we should never reach them
constexpr int32_t CARTS_ENCODERS_ZERO_VALUE = 20000;
// carts encoders have 10000 impulses per one full turn (360 deg)
// so the resolution is 360 deg/10000
constexpr float CARTS_ENCODERS_RESOLUTION = 0.036;
// constants used to read and write carts encoders files
constexpr uint32_t DELIMITER_MIDDLE_LENGTH = 1;
constexpr uint32_t DELIMITER_ENDING_LENGTH = 2;
// constants for TTControls config frame
constexpr std::uint32_t K_RESOLUTION_MULTIPLIER = 10000;
constexpr std::uint32_t KD_RESOLUTION_MULTIPLIER = 100;
constexpr std::uint32_t OVERRIDE_VELOCITY_MULTIPLIER = 100;
constexpr std::uint32_t STABILIZATION_PID_PARAMETER_MULTIPLIER = 10;
constexpr std::uint32_t ACCELERATION_RESOLUTION_MULTIPLIER = 1000;
// byte position of the encoder sum value
constexpr uint8_t SIGN_BYTE_FRAME_POSITION = 0x10;
constexpr uint8_t TRACK_ENCODER_ID_BYTE_POSITION = 0;
constexpr uint8_t TRACK_ENCODER_SIGN_BIT_MASK = 0x10;
constexpr uint8_t TRACK_ENCODER_SUM_LOW_BYTE_POSITION = 1;

constexpr uint8_t tpl_parse_constant = 110;
constexpr uint8_t engine_ready_flag_value = 1;
constexpr uint8_t engine_ready_byte = 7;

inline uint8_t TTCommControl::extractFourthByte(const uint32_t four_byte_number) const
{
  return static_cast<uint8_t>((four_byte_number & GET_FOURTH_BYTE) >> 24);
}

inline uint8_t TTCommControl::extractThirdByte(const uint32_t four_byte_number) const
{
  return static_cast<uint8_t>((four_byte_number & GET_THIRD_BYTE) >> 16);
}

inline uint8_t TTCommControl::extractSecondByte(const uint32_t four_byte_number) const
{
  return static_cast<uint8_t>((four_byte_number & GET_SECOND_BYTE) >> 8);
}

inline uint8_t TTCommControl::extractFirstByte(const uint32_t four_byte_number) const
{
  return static_cast<uint8_t>(four_byte_number & GET_FIRST_BYTE);
}

void TTCommControl::prepareVelocitiesFrame(can_frame& frame, const float velocity, const uint8_t start_byte) const
{
  int16_t velocity_to_parse = (velocity * VELOCITY_FRAME_ENCODE_CONST);

  frame.data[start_byte] = extractSecondByte(static_cast<uint32_t>(velocity_to_parse));
  // if sending velo is below zero
  if (velocity_to_parse < 0)
  {
    // perform below substraction to achive inversed number
    velocity_to_parse = INVERSE_INT16_CONST - velocity_to_parse;
    frame.data[start_byte] = extractSecondByte(static_cast<uint32_t>(velocity_to_parse));
    // toggle sign bit
    frame.data[start_byte] |= SIGN_BIT;
  }
  else
  {
    frame.data[start_byte] = extractSecondByte(static_cast<uint32_t>(velocity_to_parse));
  }
  frame.data[start_byte + 1] = extractFirstByte(static_cast<uint32_t>(velocity_to_parse));
}

float TTCommControl::decodeVelocitiy(const uint8_t start_byte, const can_frame& frame) const
{
  return (static_cast<float>((static_cast<int16_t>(frame.data[start_byte] & DATA_BITS)) << 8 |
                             frame.data[start_byte + 1])) *
         VELOCITY_FRAME_DECODE_CONST;
}

void TTCommControl::decodeCartsGeometryState(const can_frame& frame)
{
  switch (frame.data[FRONT_DELTA_GEOMETRY_BYTE])
  {
    case agribot_msgs::ChangeDeltasGeometry::Request::ELEVATED_CATERPILLAR_GEOMETRY:
      cart_.setFrontCartGeometryState(agribot_msgs::ChangeDeltasGeometry::Request::ELEVATED_CATERPILLAR_GEOMETRY);
      break;
    case agribot_msgs::ChangeDeltasGeometry::Request::NORMAL_CATERPILLAR_GEOMETRY:
      cart_.setFrontCartGeometryState(agribot_msgs::ChangeDeltasGeometry::Request::NORMAL_CATERPILLAR_GEOMETRY);
      break;
    default:
      cart_.setFrontCartGeometryState(agribot_msgs::ChangeDeltasGeometry::Request::UNKNOWN_CATERPILLAR_GEOMETRY);
      break;
  }
  switch (frame.data[BACK_DELTA_GEOMETRY_BYTE])
  {
    case agribot_msgs::ChangeDeltasGeometry::Request::ELEVATED_CATERPILLAR_GEOMETRY:
      cart_.setBackCartGeometryState(agribot_msgs::ChangeDeltasGeometry::Request::ELEVATED_CATERPILLAR_GEOMETRY);
      break;
    case agribot_msgs::ChangeDeltasGeometry::Request::NORMAL_CATERPILLAR_GEOMETRY:
      cart_.setBackCartGeometryState(agribot_msgs::ChangeDeltasGeometry::Request::NORMAL_CATERPILLAR_GEOMETRY);
      break;
    default:
      cart_.setBackCartGeometryState(agribot_msgs::ChangeDeltasGeometry::Request::UNKNOWN_CATERPILLAR_GEOMETRY);
      break;
  }
}

float TTCommControl::decodeAngle(const can_frame& frame) const
{
  return (static_cast<float>(((static_cast<int32_t>(frame.data[CARTS_ENCODER_HIGH_BYTE_POSITION])) << 8 |
                              (static_cast<int32_t>(frame.data[CARTS_ENCODER_LOW_BYTE_POSITION]))) -
                             CARTS_ENCODERS_ZERO_VALUE) *
          CARTS_ENCODERS_RESOLUTION);
}

uint32_t TTCommControl::decodeEncoderValue(const can_frame& frame) const
{
  return ((static_cast<uint32_t>(frame.data[CARTS_ENCODER_HIGH_BYTE_POSITION]) << 8) |
          (static_cast<uint32_t>(frame.data[CARTS_ENCODER_LOW_BYTE_POSITION])));
}

void TTCommControl::decodeEngineFeedback(const can_frame& frame)
{
  if (static_cast<uint16_t>(frame.data[engine_ready_byte]) == engine_ready_flag_value)
  {
    ttcontrol_engine_is_ready_ = true;
  }
  else
  {
    ttcontrol_engine_is_ready_ = false;
  }
}

inline uint8_t TTCommControl::getLowerHalfOfByte(const uint8_t byte) const
{
  return (byte & GET_LOWER_HALF_OF_THE_BYTE);
}

inline uint8_t TTCommControl::getUpperHalfOfByte(const uint8_t byte) const
{
  return (byte & GET_UPPER_HALF_OF_THE_BYTE);
}

inline uint32_t TTCommControl::parseBytesToU32Lsb(const uint8_t start_byte, const can_frame& frame) const
{
  return ((static_cast<uint32_t>(frame.data[start_byte + 3]) << 24) |
          (static_cast<uint32_t>(frame.data[start_byte + 2]) << 16) |
          (static_cast<uint32_t>(frame.data[start_byte + 1]) << 8) | (static_cast<uint32_t>(frame.data[start_byte])));
}

inline int32_t TTCommControl::checkTrackEncoderSumSignBit(const uint8_t bit_mask, const uint8_t byte_to_check) const
{
  // get the sign of the sum, if its on, then the sum is negative
  if (checkIfBitIsSet(static_cast<uint32_t>(byte_to_check), static_cast<uint32_t>(bit_mask)))
  {
    return -1;
  }
  else
  {
    return 1;
  }
}

void TTCommControl::decodeTrackEncoderSumValue(const can_frame& frame)
{
  // get the encoder sum
  uint32_t encoder_sum_without_sign = parseBytesToU32Lsb(TRACK_ENCODER_SUM_LOW_BYTE_POSITION, frame);
  // get the track ID
  uint8_t track_id = getLowerHalfOfByte(frame.data[TRACK_ENCODER_ID_BYTE_POSITION]);
  // set the decoded encoder sum value
  cart_.setTrackEncoderSum(
      static_cast<EncoderTrackId>(track_id),
      static_cast<int32_t>(encoder_sum_without_sign) *
          checkTrackEncoderSumSignBit(frame.data[TRACK_ENCODER_ID_BYTE_POSITION], TRACK_ENCODER_SIGN_BIT_MASK));
}

void TTCommControl::setVelocity(const VelocityType velocity_specification, const can_frame& frame)
{
  switch (velocity_specification)
  {
    case VelocityType::front_linear:
      cart_.setFrontLinearVelocity(decodeVelocitiy(VelocityType::front_linear, frame));
      // if there is sign bit present, change sign of velocity
      if (frame.data[VelocityType::front_linear] & SIGN_BIT)
      {
        cart_.setFrontLinearVelocity(-cart_.getFrontLinearVelocity());
      }
      break;
    case VelocityType::front_angular:
      cart_.setFrontAngularVelocity(decodeVelocitiy(VelocityType::front_angular, frame) * DEG_TO_RAD);
      if (frame.data[VelocityType::front_angular] & SIGN_BIT)
      {
        cart_.setFrontAngularVelocity(-cart_.getFrontAngularVelocity());
      }
      break;
    case VelocityType::back_linear:
      cart_.setBackLinearVelocity(decodeVelocitiy(VelocityType::back_linear, frame));
      if (frame.data[VelocityType::back_linear] & SIGN_BIT)
      {
        cart_.setBackLinearVelocity(-cart_.getBackLinearVelocity());
      }
      break;
    case VelocityType::back_angular:
      cart_.setBackAngularVelocity(decodeVelocitiy(VelocityType::back_angular, frame) * DEG_TO_RAD);
      if (frame.data[VelocityType::back_angular] & SIGN_BIT)
      {
        cart_.setBackAngularVelocity(-cart_.getBackAngularVelocity());
      }
      break;
  }
}

void TTCommControl::setTtcontrolConnectionStatus(const bool& connected)
{
  if (!connected)
  {
    ttcontrol_carts_angle_confirmed_ = false;
  }
  ttcontrol_is_connected_ = connected;
}

bool TTCommControl::getTtcontrolConnectionStatus() const
{
  return ttcontrol_is_connected_;
}

bool TTCommControl::getEngineStatus() const
{
  return ttcontrol_engine_is_ready_;
}

bool TTCommControl::getEncodersConfirmedStatus() const
{
  return ttcontrol_carts_angle_confirmed_;
}

TTCommControl::TTCommControl()
{
  ttcontrol_carts_angle_confirmed_ = false;
  ttcontrol_is_connected_ = false;
}

void TTCommControl::initializeCanHandler(std::unique_ptr<ICan> can_handler)
{
  can_handler_ = std::move(can_handler);
}

void TTCommControl::initializeCanHandler(std::string selected_can)
{
  std::unique_ptr<Can> can_handler = std::make_unique<Can>();
  can_handler->iInitialize(selected_can.c_str());
  can_handler_ = std::move(can_handler);
}

void TTCommControl::receiveCanFrames()
{
  can_frame message;
  while (can_handler_->bReceiveFrame(&message))
  {
    frames_queue_.push(message);
  }
}

TTCommControl::~TTCommControl()
{
  while (!frames_queue_.empty())
  {
    frames_queue_.pop();
  }
}

inline bool TTCommControl::checkIfCanBusIdsMatch(const uint32_t id_to_check, const uint32_t msg_id) const
{
  return ((id_to_check & TWO_LAST_BYTES_ADDRESS_MASK) == (msg_id & TWO_LAST_BYTES_ADDRESS_MASK));
}

inline void TTCommControl::setReceivedCanBusIdsBitMask(const uint32_t mask_to_set)
{
  received_msgs_register_ |= mask_to_set;
}

inline bool TTCommControl::checkIfBitIsSet(const uint32_t number_to_check, const uint32_t mask_to_check) const
{
  return ((number_to_check & mask_to_check) == mask_to_check);
}

inline void TTCommControl::clearBitInReceivedMsgsRegister(const uint32_t mask_to_check)
{
  received_msgs_register_ = (received_msgs_register_ & ~mask_to_check);
}

void TTCommControl::clearReceivedCanBusIdBit(const uint32_t mask_to_check)
{
  clearBitInReceivedMsgsRegister(mask_to_check);
}

bool TTCommControl::checkReceivedCanBusIdBit(const uint32_t mask_to_check) const
{
  return checkIfBitIsSet(received_msgs_register_, mask_to_check);
}

void TTCommControl::parseCanFrames()
{
  can_frame message;
  while (!frames_queue_.empty())
  {
    message = frames_queue_.front();
    if (checkIfCanBusIdsMatch(ReceiveCanIds::velocity_frame, message.can_id))
    {
      setVelocity(VelocityType::front_linear, message);
      setVelocity(VelocityType::front_angular, message);
      setVelocity(VelocityType::back_linear, message);
      setVelocity(VelocityType::back_angular, message);
      setReceivedCanBusIdsBitMask(ReceivedFramesRegisterMask::velocities);
    }
    if (checkIfCanBusIdsMatch(ReceiveCanIds::front_angle_frame, message.can_id))
    {
      front_cart_encoder_unread_ = true;
      front_cart_encoder_value_ = decodeEncoderValue(message);
      front_cart_angle_ = decodeAngle(message) * DEG_TO_RAD;
      setReceivedCanBusIdsBitMask(ReceivedFramesRegisterMask::front_angle);
    }
    if (checkIfCanBusIdsMatch(ReceiveCanIds::back_angle_frame, message.can_id))
    {
      back_cart_encoder_unread_ = true;
      back_cart_encoder_value_ = decodeEncoderValue(message);
      back_cart_angle_ = decodeAngle(message) * DEG_TO_RAD;
      setReceivedCanBusIdsBitMask(ReceivedFramesRegisterMask::back_angle);
    }
    if (checkIfCanBusIdsMatch(ReceiveCanIds::track_encoder_sum_frame, message.can_id))
    {
      decodeTrackEncoderSumValue(message);
      setReceivedCanBusIdsBitMask(ReceivedFramesRegisterMask::track_encoder_sum);
    }
    if (checkIfCanBusIdsMatch(ReceiveCanIds::deltas_geometry_feedback_frame, message.can_id))
    {
      decodeCartsGeometryState(message);
      setReceivedCanBusIdsBitMask(ReceivedFramesRegisterMask::geometry_feedback);
    }
    if (checkIfCanBusIdsMatch(ReceiveCanIds::engine_feedback_frame, message.can_id))
    {
      decodeEngineFeedback(message);
    }
    if (checkIfCanBusIdsMatch(ReceiveCanIds::protocol_frame, message.can_id))
    {
      // if TTControl confirms angle receiving
      if (message.data[PROTOCOL_PURPOSE_BYTE] == ProtocolFrameBytesValues::ttc_carts_encoders_initialization)
      {
        ttcontrol_carts_angle_confirmed_ = true;
      }
      // if there is a request for entering automatic mode
      if (message.data[PROTOCOL_PURPOSE_BYTE] == ProtocolFrameBytesValues::ttc_automatic_mode_request)
      {
        // if TTC said that it is ready for entering automatic mode
        if (message.data[PROTOCOL_DETAIL_BYTE] == ProtocolFrameBytesValues::ttc_ok)
        {
          setReceivedCanBusIdsBitMask(ReceivedFramesRegisterMask::automatic_state_request);
        }
        // if TTC send some error while entering automatic mode
        else
        {
          err_code_ = message.data[PROTOCOL_DETAIL_BYTE];
          setReceivedCanBusIdsBitMask(ReceivedFramesRegisterMask::automatic_state_request_error);
        }
      }
    }
    frames_queue_.pop();
  }
}

void TTCommControl::sendEngineCommand(const TTControlEngineCommands command)
{
  can_frame message;
  message.can_id = TransmitCanIds::protocol_response_frame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[0] = ProtocolFrameBytesValues::ttc_start_stop_engine;
  message.data[1] = command;  // bytes reserved for possible other engine commands
  message.data[2] = 0;
  message.data[3] = 0;
  message.data[4] = 0;
  message.data[5] = 0;
  message.data[6] = 0;
  message.data[7] = 0;
  can_handler_->iSendFrame(&message);
}

void TTCommControl::confirmAdvantechReadiness()
{
  can_frame message;
  message.can_id = TransmitCanIds::protocol_response_frame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[0] = ProtocolFrameBytesValues::ttc_automatic_mode_request;
  message.data[1] = TTCONTROL_J1939_NODE_ID;  // INFO byte, send the TTControl node ID to confirm readiness
  message.data[2] = 0;
  message.data[3] = 0;
  message.data[4] = 0;
  message.data[5] = 0;
  message.data[6] = 0;
  message.data[7] = 0;
  can_handler_->iSendFrame(&message);
}

inline uint8_t TTCommControl::returnDeltasGeometryCommandByte(const uint8_t command) const
{
  if (command != agribot_msgs::ChangeDeltasGeometry::Request::ELEVATED_CATERPILLAR_GEOMETRY &&
      command != agribot_msgs::ChangeDeltasGeometry::Request::NORMAL_CATERPILLAR_GEOMETRY)
  {
    return agribot_msgs::ChangeDeltasGeometry::Request::UNKNOWN_CATERPILLAR_GEOMETRY;
  }
  else
  {
    return command;
  }
}

void TTCommControl::sendDeltasGeometryCommand(const uint8_t front_cart_command, const uint8_t back_cart_command)
{
  can_frame message;
  message.can_id = TransmitCanIds::deltas_geometry_command_frame;
  message.can_dlc = TWO_BYTE_CAN_FRAME;
  message.data[FRONT_DELTA_GEOMETRY_BYTE] = returnDeltasGeometryCommandByte(front_cart_command);
  message.data[BACK_DELTA_GEOMETRY_BYTE] = returnDeltasGeometryCommandByte(back_cart_command);
  can_handler_->iSendFrame(&message);
}

void TTCommControl::sendCartsEncodersInitialization()
{
  can_frame message;
  message.can_id = TransmitCanIds::protocol_response_frame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[0] = ProtocolFrameBytesValues::ttc_carts_encoders_initialization;
  message.data[1] = extractThirdByte(init_front_cart_encoder_value_);
  message.data[2] = extractSecondByte(init_front_cart_encoder_value_);
  message.data[3] = extractFirstByte(init_front_cart_encoder_value_);
  message.data[4] = extractThirdByte(init_back_cart_encoder_value_);
  message.data[5] = extractSecondByte(init_back_cart_encoder_value_);
  message.data[6] = extractFirstByte(init_back_cart_encoder_value_);
  message.data[7] = 0;
  can_handler_->iSendFrame(&message);
}

void TTCommControl::sendVelocities(const float front_linear, const float front_angular, const float back_linear,
                                   const float back_angular)
{
  can_frame message;
  message.can_id = TransmitCanIds::velocity_control_frame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  prepareVelocitiesFrame(message, front_linear, 0);
  prepareVelocitiesFrame(message, front_angular, 2);
  prepareVelocitiesFrame(message, back_linear, 4);
  prepareVelocitiesFrame(message, back_angular, 6);
  can_handler_->iSendFrame(&message);
}

void TTCommControl::sendSignalsParameters(const int ditter_period, const int ditter_amplitude, const int encoder_period)
{
  can_frame message;
  message.can_id = TransmitCanIds::config_frame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[config_frame_type_byte] = TTControlParameterType::signals;
  message.data[SignalsParametersBytePosition::ditter_period_byte] =
      extractFirstByte(static_cast<uint32_t>(ditter_period));
  message.data[SignalsParametersBytePosition::ditter_amplitude_first_byte] =
      extractFirstByte(static_cast<uint32_t>(ditter_amplitude));
  message.data[SignalsParametersBytePosition::ditter_amplitude_first_byte + 1] =
      extractSecondByte(static_cast<uint32_t>(ditter_amplitude));
  message.data[SignalsParametersBytePosition::encoder_period_byte] =
      extractFirstByte(static_cast<uint32_t>(encoder_period));
  // dont-care bytes
  message.data[SignalsParametersBytePosition::encoder_period_byte + 1] = 0xFF;
  message.data[SignalsParametersBytePosition::encoder_period_byte + 2] = 0xFF;
  message.data[SignalsParametersBytePosition::encoder_period_byte + 3] = 0xFF;
  can_handler_->iSendFrame(&message);
}

void TTCommControl::sendCartsPidParameters(const double kp, const double ki, const double kd, const double acceleration,
                                           const uint8_t frame_type)
{
  can_frame message;
  message.can_id = TransmitCanIds::config_frame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[config_frame_type_byte] = frame_type;
  message.data[CartsPidParametersBytePositions::kp_first_byte] =
      extractFirstByte(static_cast<uint32_t>(kp * K_RESOLUTION_MULTIPLIER));
  message.data[CartsPidParametersBytePositions::kp_first_byte + 1] =
      extractSecondByte(static_cast<uint32_t>(kp * K_RESOLUTION_MULTIPLIER));
  message.data[CartsPidParametersBytePositions::ki_first_byte] =
      extractFirstByte(static_cast<uint32_t>(ki * K_RESOLUTION_MULTIPLIER));
  message.data[CartsPidParametersBytePositions::ki_first_byte + 1] =
      extractSecondByte(static_cast<uint32_t>(ki * K_RESOLUTION_MULTIPLIER));
  message.data[CartsPidParametersBytePositions::kd_first_byte] =
      extractFirstByte(static_cast<uint32_t>(kd * KD_RESOLUTION_MULTIPLIER));
  message.data[CartsPidParametersBytePositions::acceleration_first_byte] =
      extractFirstByte(static_cast<uint32_t>(acceleration * ACCELERATION_RESOLUTION_MULTIPLIER));
  message.data[CartsPidParametersBytePositions::acceleration_first_byte + 1] =
      extractSecondByte(static_cast<uint32_t>(acceleration * ACCELERATION_RESOLUTION_MULTIPLIER));
  can_handler_->iSendFrame(&message);
}

void TTCommControl::sendCaterpillarPidParameters(const uint32_t kp, const uint32_t ki, const uint32_t kd,
                                                 const uint8_t frame_type)
{
  can_frame message;
  message.can_id = TransmitCanIds::config_frame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[config_frame_type_byte] = frame_type;
  message.data[CaterpillarPidParametersBytePosition::kp_caterpillar_first_byte] = extractFirstByte(kp);
  message.data[CaterpillarPidParametersBytePosition::kp_caterpillar_first_byte + 1] = extractSecondByte(kp);
  message.data[CaterpillarPidParametersBytePosition::ki_caterpillar_first_byte] = extractFirstByte(ki);
  message.data[CaterpillarPidParametersBytePosition::ki_caterpillar_first_byte + 1] = extractSecondByte(ki);
  message.data[CaterpillarPidParametersBytePosition::kd_caterpillar_first_byte] = extractFirstByte(kd);
  message.data[CaterpillarPidParametersBytePosition::kd_caterpillar_first_byte + 1] = extractSecondByte(kd);
  // dont care byte
  message.data[CaterpillarPidParametersBytePosition::kd_caterpillar_first_byte + 2] = 0xFF;
  can_handler_->iSendFrame(&message);
}

void TTCommControl::sendCaterpillarBaseSignal(const uint32_t forward_base, const uint32_t backward_base,
                                              const uint8_t frame_type)
{
  can_frame message;
  message.can_id = TransmitCanIds::config_frame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[config_frame_type_byte] = frame_type;
  message.data[SignalBaseBytePositions::forward_signal_base_first_byte] = extractFirstByte(forward_base);
  message.data[SignalBaseBytePositions::forward_signal_base_first_byte + 1] = extractSecondByte(forward_base);
  message.data[SignalBaseBytePositions::backward_signal_base_first_byte] = extractFirstByte(backward_base);
  message.data[SignalBaseBytePositions::backward_signal_base_first_byte + 1] = extractSecondByte(backward_base);
  // dont care bytes
  message.data[SignalBaseBytePositions::backward_signal_base_first_byte + 2] = 0xFF;
  message.data[SignalBaseBytePositions::backward_signal_base_first_byte + 3] = 0xFF;
  message.data[SignalBaseBytePositions::backward_signal_base_first_byte + 4] = 0xFF;
  can_handler_->iSendFrame(&message);
}

void TTCommControl::sendOverrideLimits(const TtcontrolOverrideParameters& params)
{
  can_frame message;
  message.can_id = TransmitCanIds::config_frame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[config_frame_type_byte] = TTControlParameterType::override_speed_and_valve_limits;
  message.data[OverrideAndValvesLimitsBytePositions::cart_rotation_limit_byte] = params.cart_rotation_limit;
  message.data[OverrideAndValvesLimitsBytePositions::override_linear_velocity_byte] =
      static_cast<uint8_t>(params.override_velocity_linear * OVERRIDE_VELOCITY_MULTIPLIER);
  message.data[OverrideAndValvesLimitsBytePositions::override_angular_velocity_byte] =
      static_cast<uint8_t>(params.override_velocity_angular * OVERRIDE_VELOCITY_MULTIPLIER);
  message.data[OverrideAndValvesLimitsBytePositions::override_valves_limits_byte] = params.override_valves_limits;
  message.data[OverrideAndValvesLimitsBytePositions::deltas_geometry_valves_push_limit_front_byte] =
      params.deltas_geometry_valves_push_limit_front;
  message.data[OverrideAndValvesLimitsBytePositions::deltas_geometry_valves_push_limit_back_byte] =
      params.deltas_geometry_valves_push_limit_back;
  message.data[OverrideAndValvesLimitsBytePositions::deltas_geometry_valves_pull_front_back_byte] =
      params.deltas_geometry_valves_pull_front_back;
  can_handler_->iSendFrame(&message);
}

void TTCommControl::sendStabilizationParameters(const double stab_kp, const double stab_kd, const double stab_ki,
                                                const std::uint8_t stab_max_signal)
{
  can_frame message;
  message.can_id = TransmitCanIds::config_frame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[config_frame_type_byte] = TTControlParameterType::stabilization;
  message.data[StabilizationBytePositions::stabilization_kp_byte] =
      static_cast<uint8_t>(stab_kp * STABILIZATION_PID_PARAMETER_MULTIPLIER);
  message.data[StabilizationBytePositions::stabilization_kd_byte] =
      static_cast<uint8_t>(stab_kd * STABILIZATION_PID_PARAMETER_MULTIPLIER);
  message.data[StabilizationBytePositions::stabilization_ki_byte] =
      static_cast<uint8_t>(stab_ki * STABILIZATION_PID_PARAMETER_MULTIPLIER);
  message.data[StabilizationBytePositions::stabilization_maximum_signal_byte] = stab_max_signal;
  message.data[StabilizationBytePositions::stabilization_maximum_signal_byte + 1] = 0;
  message.data[StabilizationBytePositions::stabilization_maximum_signal_byte + 2] = 0;
  message.data[StabilizationBytePositions::stabilization_maximum_signal_byte + 3] = 0;
  can_handler_->iSendFrame(&message);
}

void TTCommControl::sendOilAndEngineParams(const std::uint8_t max_oil_temperature,
                                           const std::uint8_t min_oil_temperature,
                                           const std::uint8_t histeresis_oil_temperature, const std::uint16_t rpm_max,
                                           const std::uint16_t rpm_min)
{
  can_frame message;
  message.can_id = TransmitCanIds::config_frame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[config_frame_type_byte] = TTControlParameterType::oil_and_engine;
  message.data[OilAndEngineBytePositions::max_oil_temperature_byte] = max_oil_temperature;
  message.data[OilAndEngineBytePositions::min_oil_temperature_byte] = min_oil_temperature;
  message.data[OilAndEngineBytePositions::histeresis_oil_temperature_byte] = histeresis_oil_temperature;
  message.data[OilAndEngineBytePositions::rpm_max_byte] = extractFirstByte(rpm_max);
  message.data[OilAndEngineBytePositions::rpm_max_byte + 1] = extractSecondByte(rpm_max);
  message.data[OilAndEngineBytePositions::rpm_min_byte] = extractFirstByte(rpm_min);
  message.data[OilAndEngineBytePositions::rpm_min_byte + 1] = extractSecondByte(rpm_min);
  can_handler_->iSendFrame(&message);
}

const CartsStatus& TTCommControl::getCartsStatus() const
{
  return cart_;
}

float TTCommControl::getFrontCartAngle() const
{
  return front_cart_angle_;
}

float TTCommControl::getBackCartAngle() const
{
  return back_cart_angle_;
}

std::int32_t TTCommControl::getErrorCode() const
{
  return err_code_;
}

void TTCommControl::setPathToCartsEncodersDataFile(const std::string& path_to_file)
{
  path_to_encoder_file_ = path_to_file;
}

uint32_t TTCommControl::extractEncoderInitValueFromFile(std::ifstream& file_input) const
{
  const char delimiter_middle = ':';
  const char delimiter_endline = ';';
  std::string line;
  std::string number_str;
  std::getline(file_input, line);
  if (line.size() > 0)
  {
    // check if middle delimiter is in the string, without it, the substr function
    // can return exception because find function return npos
    if (line.find(delimiter_middle) == std::string::npos)
    {
      return 0;
    }
    number_str = line.substr(line.find(delimiter_middle));
    if (number_str.front() == delimiter_middle && number_str.back() == delimiter_endline)
    {
      number_str = number_str.substr(DELIMITER_MIDDLE_LENGTH, number_str.length() - DELIMITER_ENDING_LENGTH);
      return static_cast<uint32_t>(std::stoul(number_str));
    }
    else
    {
      return 0;
    }
  }
  else
  {
    return 0;
  }
}

bool TTCommControl::readCartsEncodersDataFile(const bool regular_file)
{
  std::string encoder_file_name;

  if (regular_file)
  {
    encoder_file_name = file_name_;
  }
  else
  {
    encoder_file_name = file_name_backup_;
  }
  encoder_data_file_input_.open(path_to_encoder_file_ + encoder_file_name);
  if (!encoder_data_file_input_.is_open() || encoder_data_file_input_.fail())
  {
    encoder_data_file_input_.close();
    return false;
  }
  // first line for the front cart encoder value
  init_front_cart_encoder_value_ = extractEncoderInitValueFromFile(encoder_data_file_input_);
  if (init_front_cart_encoder_value_ == 0)
  {
    encoder_data_file_input_.close();
    return false;
  }

  // second line for the back cart encoder value
  init_back_cart_encoder_value_ = extractEncoderInitValueFromFile(encoder_data_file_input_);
  if (init_back_cart_encoder_value_ == 0)
  {
    encoder_data_file_input_.close();
    return false;
  }

  encoder_data_file_input_.close();
  return true;
}

bool TTCommControl::writeEncodersDataFile()
{
  // check if angle data is complete
  if (front_cart_encoder_unread_ && back_cart_encoder_unread_)
  {
    // save the proper file
    encoder_data_file_output_.open((path_to_encoder_file_ + file_name_), std::ios::trunc);
    if (!encoder_data_file_output_.is_open())
    {
      encoder_data_file_output_.close();
      return false;
    }
    encoder_data_file_output_ << "front:" << front_cart_encoder_value_ << ";\n";
    encoder_data_file_output_ << "back:" << back_cart_encoder_value_ << ";";
    encoder_data_file_output_.close();

    // save the backup file
    encoder_data_file_output_.open((path_to_encoder_file_ + file_name_backup_), std::ios::trunc);
    if (!encoder_data_file_output_.is_open())
    {
      encoder_data_file_output_.close();
      return false;
    }
    encoder_data_file_output_ << "front:" << front_cart_encoder_value_ << ";\n";
    encoder_data_file_output_ << "back:" << back_cart_encoder_value_ << ";";
    encoder_data_file_output_.close();
    // reset flags
    front_cart_encoder_unread_ = false;
    back_cart_encoder_unread_ = false;
  }

  return true;
}

void TTCommControl::prepareTplControlFrame(const SelectedTpl selection, const TplControl& control_values,
                                           can_frame& message) const
{
  switch (selection)
  {
    case SelectedTpl::front:
      message.can_id = TransmitCanIds::tpl_control_front;
      break;
    case SelectedTpl::back:
      message.can_id = TransmitCanIds::tpl_control_back;
      break;
  }
  message.can_dlc = TPL_CONTROL_FRAME_SIZE;
  message.data[0] = control_values.pto_valve + tpl_parse_constant;
  message.data[1] = control_values.red_qcf_valve + tpl_parse_constant;
  message.data[2] = control_values.green_qcf_valve + tpl_parse_constant;
  message.data[3] = control_values.blue_qcf_valve + tpl_parse_constant;
  message.data[4] = control_values.height_actuator + tpl_parse_constant;
  message.data[5] = control_values.angle_actuator + tpl_parse_constant;
}

void TTCommControl::sendTplValveControlViaCan(const SelectedTpl selection, const TplControl& control_values) const
{
  can_frame message;
  prepareTplControlFrame(selection, control_values, message);
  can_handler_->iSendFrame(&message);
}
