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
// byte positions of the ttcontrol state frame
constexpr uint8_t AUTOMATIC_STATE_BYTE = 0x00;
constexpr uint8_t MANUAL_STATE_BYTE = 0x01;
constexpr uint8_t ENCODERS_INITED_BYTE = 0x02;
constexpr uint8_t ENCODERS_INITED_FLAG_VALUE = 0x01;

constexpr uint8_t TPL_PARSE_CONSTANT = 110;
constexpr uint8_t ENGINE_READY_FLAG_VALUE = 1;
constexpr uint8_t ENGINE_READY_BYTE = 7;

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
  if (static_cast<uint16_t>(frame.data[ENGINE_READY_BYTE]) == ENGINE_READY_FLAG_VALUE)
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

inline void TTCommControl::decodeTtcontrolState(const can_frame& frame)
{
  ttcontrol_current_state_.automatic_state = static_cast<TtcontrolState>(frame.data[AUTOMATIC_STATE_BYTE]);
  ttcontrol_current_state_.manual_state = static_cast<TtcontrolState>(frame.data[MANUAL_STATE_BYTE]);
  if (frame.data[ENCODERS_INITED_BYTE] == ENCODERS_INITED_FLAG_VALUE)
  {
    ttcontrol_carts_angle_confirmed_ = true;
  }
  else
  {
    ttcontrol_carts_angle_confirmed_ = false;
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
    case VelocityType::FrontLinear:
      cart_.setFrontLinearVelocity(decodeVelocitiy(VelocityType::FrontLinear, frame));
      // if there is sign bit present, change sign of velocity
      if (frame.data[VelocityType::FrontLinear] & SIGN_BIT)
      {
        cart_.setFrontLinearVelocity(-cart_.getFrontLinearVelocity());
      }
      break;
    case VelocityType::FrontAngular:
      cart_.setFrontAngularVelocity(decodeVelocitiy(VelocityType::FrontAngular, frame) * DEG_TO_RAD);
      if (frame.data[VelocityType::FrontAngular] & SIGN_BIT)
      {
        cart_.setFrontAngularVelocity(-cart_.getFrontAngularVelocity());
      }
      break;
    case VelocityType::BackLinear:
      cart_.setBackLinearVelocity(decodeVelocitiy(VelocityType::BackLinear, frame));
      if (frame.data[VelocityType::BackLinear] & SIGN_BIT)
      {
        cart_.setBackLinearVelocity(-cart_.getBackLinearVelocity());
      }
      break;
    case VelocityType::BackAngular:
      cart_.setBackAngularVelocity(decodeVelocitiy(VelocityType::BackAngular, frame) * DEG_TO_RAD);
      if (frame.data[VelocityType::BackAngular] & SIGN_BIT)
      {
        cart_.setBackAngularVelocity(-cart_.getBackAngularVelocity());
      }
      break;
  }
}

bool TTCommControl::getTtcontrolConnectionStatus() const
{
  if (ttcontrol_current_state_.automatic_state == TtcontrolState::CommunicationActive ||
      ttcontrol_current_state_.automatic_state == TtcontrolState::SteeringActive)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool TTCommControl::getEngineStatus() const
{
  return ttcontrol_engine_is_ready_;
}

bool TTCommControl::getEncodersConfirmedStatus() const
{
  return ttcontrol_carts_angle_confirmed_;
}

TtcontrolCurrentStates TTCommControl::getTtcontrolCurrentState() const
{
  return ttcontrol_current_state_;
}

TTCommControl::TTCommControl()
{
  ttcontrol_carts_angle_confirmed_ = false;
  ttcontrol_is_connected_ = false;
  ttcontrol_current_state_.automatic_state = TtcontrolState::Initialization;
  ttcontrol_current_state_.manual_state = TtcontrolState::Initialization;
}

void TTCommControl::initializeCanHandler(std::unique_ptr<ICan> can_handler)
{
  can_handler_ = std::move(can_handler);
}

void TTCommControl::initializeCanHandler(const std::string& selected_can)
{
  can_handler_ = std::make_unique<Can>();
  can_handler_->iInitialize(selected_can.c_str());
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
    if (checkIfCanBusIdsMatch(ReceiveCanIds::VelocityFrame, message.can_id))
    {
      setVelocity(VelocityType::FrontLinear, message);
      setVelocity(VelocityType::FrontAngular, message);
      setVelocity(VelocityType::BackLinear, message);
      setVelocity(VelocityType::BackAngular, message);
      setReceivedCanBusIdsBitMask(ReceivedFramesRegisterMask::Velocities);
    }
    if (checkIfCanBusIdsMatch(ReceiveCanIds::FrontAngleFrame, message.can_id))
    {
      front_cart_encoder_unread_ = true;
      front_cart_encoder_value_ = decodeEncoderValue(message);
      front_cart_angle_ = decodeAngle(message) * DEG_TO_RAD;
      setReceivedCanBusIdsBitMask(ReceivedFramesRegisterMask::FrontAngle);
    }
    if (checkIfCanBusIdsMatch(ReceiveCanIds::BackAngleFrame, message.can_id))
    {
      back_cart_encoder_unread_ = true;
      back_cart_encoder_value_ = decodeEncoderValue(message);
      back_cart_angle_ = decodeAngle(message) * DEG_TO_RAD;
      setReceivedCanBusIdsBitMask(ReceivedFramesRegisterMask::BackAngle);
    }
    if (checkIfCanBusIdsMatch(ReceiveCanIds::TrackEncoderSumFrame, message.can_id))
    {
      decodeTrackEncoderSumValue(message);
      setReceivedCanBusIdsBitMask(ReceivedFramesRegisterMask::TrackEncoderSum);
    }
    if (checkIfCanBusIdsMatch(ReceiveCanIds::DeltasGeometryFeedbackFrame, message.can_id))
    {
      decodeCartsGeometryState(message);
      setReceivedCanBusIdsBitMask(ReceivedFramesRegisterMask::GeometryFeedback);
    }
    if (checkIfCanBusIdsMatch(ReceiveCanIds::EngineFeedbackFrame, message.can_id))
    {
      decodeEngineFeedback(message);
    }
    if (checkIfCanBusIdsMatch(ReceiveCanIds::TtcontrolStateFrame, message.can_id))
    {
      decodeTtcontrolState(message);
      setReceivedCanBusIdsBitMask(ReceivedFramesRegisterMask::TtcontrolStates);
    }
    if (checkIfCanBusIdsMatch(ReceiveCanIds::ProtocolFrame, message.can_id))
    {
      // if TTControl confirms angle receiving
      if (message.data[PROTOCOL_PURPOSE_BYTE] == ProtocolFrameBytesValues::TtcCartsEncodersInitialization)
      {
        ttcontrol_carts_angle_confirmed_ = true;
      }
      // if there is a request for entering automatic mode
      if (message.data[PROTOCOL_PURPOSE_BYTE] == ProtocolFrameBytesValues::TtcAutomaticModeRequest)
      {
        // if TTC said that it is ready for entering automatic mode
        if (message.data[PROTOCOL_DETAIL_BYTE] == ProtocolFrameBytesValues::TtcOk)
        {
          setReceivedCanBusIdsBitMask(ReceivedFramesRegisterMask::AutomaticStateRequest);
        }
        // if TTC send some error while entering automatic mode
        else
        {
          err_code_ = message.data[PROTOCOL_DETAIL_BYTE];
          setReceivedCanBusIdsBitMask(ReceivedFramesRegisterMask::AutomaticStateRequestError);
        }
      }
    }
    frames_queue_.pop();
  }
}

void TTCommControl::sendEngineCommand(const TTControlEngineCommands command)
{
  can_frame message;
  message.can_id = TransmitCanIds::ProtocolResponseFrame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[0] = ProtocolFrameBytesValues::TtcStartStopEngine;
  message.data[1] = command;  // bytes reserved for possible other engine commands
  message.data[2] = 0;
  message.data[3] = 0;
  message.data[4] = 0;
  message.data[5] = 0;
  message.data[6] = 0;
  message.data[7] = 0;
  can_handler_->iSendFrame(&message);
}

void TTCommControl::requestChangeOfControlState(TtcontrolAutomaticModeRequestByteValues request)
{
  can_frame message;
  message.can_id = TransmitCanIds::ProtocolResponseFrame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[ProtocolFrameBytePositions::FrameTypeByte] = ProtocolFrameBytesValues::TtcAutomaticModeRequest;
  message.data[ProtocolFrameBytePositions::TtcontrolIDByte] = TTCONTROL_J1939_NODE_ID;
  message.data[ProtocolFrameBytePositions::RequestTtcontrolStateByte] = request;
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
  message.can_id = TransmitCanIds::DeltasGeometryCommandFrame;
  message.can_dlc = TWO_BYTE_CAN_FRAME;
  message.data[FRONT_DELTA_GEOMETRY_BYTE] = returnDeltasGeometryCommandByte(front_cart_command);
  message.data[BACK_DELTA_GEOMETRY_BYTE] = returnDeltasGeometryCommandByte(back_cart_command);
  can_handler_->iSendFrame(&message);
}

void TTCommControl::sendCartsEncodersInitialization()
{
  can_frame message;
  message.can_id = TransmitCanIds::ProtocolResponseFrame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[0] = ProtocolFrameBytesValues::TtcCartsEncodersInitialization;
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
  message.can_id = TransmitCanIds::VelocityControlFrame;
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
  message.can_id = TransmitCanIds::ConfigFrame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[config_frame_type_byte] = TTControlParameterType::Signals;
  message.data[SignalsParametersBytePosition::DitterPeriodByte] =
      extractFirstByte(static_cast<uint32_t>(ditter_period));
  message.data[SignalsParametersBytePosition::DitterAmplitudeFirstByte] =
      extractFirstByte(static_cast<uint32_t>(ditter_amplitude));
  message.data[SignalsParametersBytePosition::DitterAmplitudeFirstByte + 1] =
      extractSecondByte(static_cast<uint32_t>(ditter_amplitude));
  message.data[SignalsParametersBytePosition::EncoderPeriodByte] =
      extractFirstByte(static_cast<uint32_t>(encoder_period));
  // dont-care bytes
  message.data[SignalsParametersBytePosition::EncoderPeriodByte + 1] = 0xFF;
  message.data[SignalsParametersBytePosition::EncoderPeriodByte + 2] = 0xFF;
  message.data[SignalsParametersBytePosition::EncoderPeriodByte + 3] = 0xFF;
  can_handler_->iSendFrame(&message);
}

void TTCommControl::sendCartsPidParameters(const double kp, const double ki, const double kd, const double acceleration,
                                           const uint8_t frame_type)
{
  can_frame message;
  message.can_id = TransmitCanIds::ConfigFrame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[config_frame_type_byte] = frame_type;
  message.data[CartsPidParametersBytePositions::KpFirstByte] =
      extractFirstByte(static_cast<uint32_t>(kp * K_RESOLUTION_MULTIPLIER));
  message.data[CartsPidParametersBytePositions::KpFirstByte + 1] =
      extractSecondByte(static_cast<uint32_t>(kp * K_RESOLUTION_MULTIPLIER));
  message.data[CartsPidParametersBytePositions::KiFirstByte] =
      extractFirstByte(static_cast<uint32_t>(ki * K_RESOLUTION_MULTIPLIER));
  message.data[CartsPidParametersBytePositions::KiFirstByte + 1] =
      extractSecondByte(static_cast<uint32_t>(ki * K_RESOLUTION_MULTIPLIER));
  message.data[CartsPidParametersBytePositions::KdFirstByte] =
      extractFirstByte(static_cast<uint32_t>(kd * KD_RESOLUTION_MULTIPLIER));
  message.data[CartsPidParametersBytePositions::AccelerationFirstByte] =
      extractFirstByte(static_cast<uint32_t>(acceleration * ACCELERATION_RESOLUTION_MULTIPLIER));
  message.data[CartsPidParametersBytePositions::AccelerationFirstByte + 1] =
      extractSecondByte(static_cast<uint32_t>(acceleration * ACCELERATION_RESOLUTION_MULTIPLIER));
  can_handler_->iSendFrame(&message);
}

void TTCommControl::sendCaterpillarPidParameters(const uint32_t kp, const uint32_t ki, const uint32_t kd,
                                                 const uint8_t frame_type)
{
  can_frame message;
  message.can_id = TransmitCanIds::ConfigFrame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[config_frame_type_byte] = frame_type;
  message.data[CaterpillarPidParametersBytePosition::KpCaterpillarFirstByte] = extractFirstByte(kp);
  message.data[CaterpillarPidParametersBytePosition::KpCaterpillarFirstByte + 1] = extractSecondByte(kp);
  message.data[CaterpillarPidParametersBytePosition::KiCaterpillarFirstByte] = extractFirstByte(ki);
  message.data[CaterpillarPidParametersBytePosition::KiCaterpillarFirstByte + 1] = extractSecondByte(ki);
  message.data[CaterpillarPidParametersBytePosition::KdCaterpillarFirstByte] = extractFirstByte(kd);
  message.data[CaterpillarPidParametersBytePosition::KdCaterpillarFirstByte + 1] = extractSecondByte(kd);
  // dont care byte
  message.data[CaterpillarPidParametersBytePosition::KdCaterpillarFirstByte + 2] = 0xFF;
  can_handler_->iSendFrame(&message);
}

void TTCommControl::sendCaterpillarBaseSignal(const uint32_t forward_base, const uint32_t backward_base,
                                              const uint8_t frame_type)
{
  can_frame message;
  message.can_id = TransmitCanIds::ConfigFrame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[config_frame_type_byte] = frame_type;
  message.data[SignalBaseBytePositions::ForwardSignalBaseFirstByte] = extractFirstByte(forward_base);
  message.data[SignalBaseBytePositions::ForwardSignalBaseFirstByte + 1] = extractSecondByte(forward_base);
  message.data[SignalBaseBytePositions::BackwardSignalBaseFirstByte] = extractFirstByte(backward_base);
  message.data[SignalBaseBytePositions::BackwardSignalBaseFirstByte + 1] = extractSecondByte(backward_base);
  // dont care bytes
  message.data[SignalBaseBytePositions::BackwardSignalBaseFirstByte + 2] = 0xFF;
  message.data[SignalBaseBytePositions::BackwardSignalBaseFirstByte + 3] = 0xFF;
  message.data[SignalBaseBytePositions::BackwardSignalBaseFirstByte + 4] = 0xFF;
  can_handler_->iSendFrame(&message);
}

void TTCommControl::sendOverrideLimits(const TtcontrolOverrideParameters& params)
{
  can_frame message;
  message.can_id = TransmitCanIds::ConfigFrame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[config_frame_type_byte] = TTControlParameterType::OverrideSpeedAndValveLimits;
  message.data[OverrideAndValvesLimitsBytePositions::CartRotationLimitByte] = params.cart_rotation_limit;
  message.data[OverrideAndValvesLimitsBytePositions::OverrideLinearVelocityByte] =
      static_cast<uint8_t>(params.override_velocity_linear * OVERRIDE_VELOCITY_MULTIPLIER);
  message.data[OverrideAndValvesLimitsBytePositions::OverrideAngularVelocityByte] =
      static_cast<uint8_t>(params.override_velocity_angular * OVERRIDE_VELOCITY_MULTIPLIER);
  message.data[OverrideAndValvesLimitsBytePositions::OverrideValvesLimitsByte] = params.override_valves_limits;
  message.data[OverrideAndValvesLimitsBytePositions::DeltasGeometryValvesPushLimitFrontByte] =
      params.deltas_geometry_valves_push_limit_front;
  message.data[OverrideAndValvesLimitsBytePositions::DeltasGeometryValvesPushLimitBackByte] =
      params.deltas_geometry_valves_push_limit_back;
  message.data[OverrideAndValvesLimitsBytePositions::DeltasGeometryValvesPullFrontBackByte] =
      params.deltas_geometry_valves_pull_front_back;
  can_handler_->iSendFrame(&message);
}

void TTCommControl::sendStabilizationParameters(const double stab_kp, const double stab_kd, const double stab_ki,
                                                const std::uint8_t stab_max_signal)
{
  can_frame message;
  message.can_id = TransmitCanIds::ConfigFrame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[config_frame_type_byte] = TTControlParameterType::Stabilization;
  message.data[StabilizationBytePositions::StabilizationKpByte] =
      static_cast<uint8_t>(stab_kp * STABILIZATION_PID_PARAMETER_MULTIPLIER);
  message.data[StabilizationBytePositions::StabilizationKdByte] =
      static_cast<uint8_t>(stab_kd * STABILIZATION_PID_PARAMETER_MULTIPLIER);
  message.data[StabilizationBytePositions::StabilizationKiByte] =
      static_cast<uint8_t>(stab_ki * STABILIZATION_PID_PARAMETER_MULTIPLIER);
  message.data[StabilizationBytePositions::StabilizationMaximumSignalByte] = stab_max_signal;
  message.data[StabilizationBytePositions::StabilizationMaximumSignalByte + 1] = 0;
  message.data[StabilizationBytePositions::StabilizationMaximumSignalByte + 2] = 0;
  message.data[StabilizationBytePositions::StabilizationMaximumSignalByte + 3] = 0;
  can_handler_->iSendFrame(&message);
}

void TTCommControl::sendOilAndEngineParams(const std::uint8_t max_oil_temperature,
                                           const std::uint8_t min_oil_temperature,
                                           const std::uint8_t histeresis_oil_temperature, const std::uint16_t rpm_max,
                                           const std::uint16_t rpm_min)
{
  can_frame message;
  message.can_id = TransmitCanIds::ConfigFrame;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[config_frame_type_byte] = TTControlParameterType::OilAndEngine;
  message.data[OilAndEngineBytePositions::MaxOilTemperatureByte] = max_oil_temperature;
  message.data[OilAndEngineBytePositions::MinOilTemperatureByte] = min_oil_temperature;
  message.data[OilAndEngineBytePositions::HisteresisOilTemperatureByte] = histeresis_oil_temperature;
  message.data[OilAndEngineBytePositions::RpmMaxByte] = extractFirstByte(rpm_max);
  message.data[OilAndEngineBytePositions::RpmMaxByte + 1] = extractSecondByte(rpm_max);
  message.data[OilAndEngineBytePositions::RpmMinByte] = extractFirstByte(rpm_min);
  message.data[OilAndEngineBytePositions::RpmMinByte + 1] = extractSecondByte(rpm_min);
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
  // during ttcontrol programming when communication is active, from undefined reason 0, 0 values are send BEFORE
  // the can bus connection is severed, this is a counter measure against this behaviour,
  // to protect file from being corrupted
  if (front_cart_encoder_value_ != 0 && back_cart_encoder_value_ != 0)
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
  }
  return true;
}

void TTCommControl::prepareTplControlFrame(const SelectedTpl selection, const TplControl& control_values,
                                           can_frame& message) const
{
  switch (selection)
  {
    case SelectedTpl::Front:
      message.can_id = TransmitCanIds::TplControlFront;
      break;
    case SelectedTpl::Back:
      message.can_id = TransmitCanIds::TplControlBack;
      break;
  }
  message.can_dlc = TPL_CONTROL_FRAME_SIZE;
  message.data[0] = control_values.pto_valve + TPL_PARSE_CONSTANT;
  message.data[1] = control_values.red_qcf_valve + TPL_PARSE_CONSTANT;
  message.data[2] = control_values.green_qcf_valve + TPL_PARSE_CONSTANT;
  message.data[3] = control_values.blue_qcf_valve + TPL_PARSE_CONSTANT;
  message.data[4] = control_values.height_actuator + TPL_PARSE_CONSTANT;
  message.data[5] = control_values.angle_actuator + TPL_PARSE_CONSTANT;
}

void TTCommControl::sendTplValveControlViaCan(const SelectedTpl selection, const TplControl& control_values) const
{
  can_frame message;
  prepareTplControlFrame(selection, control_values, message);
  can_handler_->iSendFrame(&message);
}
