/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <hal_ttcontrol_communication/TTCommControl.h>

// mask used for extracting address and source, without PG
constexpr uint8_t TTCONTROL_J1939_NODE_ID = 0x64;
constexpr uint ADDR_MASK = 0xFFFF;
constexpr uint8_t FULL_CAN_FRAME_BYTE_NUMBER = 8;
constexpr uint8_t TWO_BYTE_CAN_FRAME = 2;
constexpr uint PROTOCOL_FRAME_ID = 0x00EFC864;
constexpr uint PROTOCOL_RESPONSE_FRAME_ID = 0x00EF64C8;
constexpr uint VELO_FRAME_ID = 0x00FFD064;
constexpr uint FRONT_ANGLE_ID = 0xAA20;
constexpr uint BACK_ANGLE_ID = 0xAA21;
constexpr uint DELTAS_GEOMETRY_FEEDBACK_ID = 0x00FFDC64;
constexpr uint DELTAS_GEOMETRY_COMMAND_ID = 0x00FFDCC8;
constexpr uint VELOCITY_CONTROL_FRAME_ID = 0x00FFD0C8;
constexpr uint CONFIG_FRAME_CAN_ID = 0x00FFDBC8;
constexpr double DEG_TO_RAD = (M_PI / 180.0);

constexpr uint SIGN_BIT = 0x80;
constexpr uint DATA_BITS = 0x7F;

// protocol data bytes
constexpr uint8_t PROTOCOL_PURPOSE_BYTE = 0;
constexpr uint8_t PROTOCOL_DETAIL_BYTE = 1;

// deltas geometry steering and feedback
constexpr uint8_t FRONT_DELTA_GEOMETRY_BYTE = 0;
constexpr uint8_t BACK_DELTA_GEOMETRY_BYTE = 1;
constexpr bool DELTAS_GEOMETRY_UP = true;
constexpr bool DELTAS_GEOMETRY_DOWN = false;
// Encode velocity frame coming from TTC - check TTC-Advantech
// protocol DS for more info
constexpr uint VELO_FRAME_ENCODE_CONST = 1000;
// Decode velocity frame coming from TTC - check TTC-Advantech
// protocol DS for more info
constexpr float VELO_FRAME_DECODE_CONST = 0.001;
// Decode angle frame - check inclinometer DS for more info
constexpr float ANGLE_FRAME_DECODE_CONST_MULTIPLIER = 0.214;
constexpr float ANGLE_FRAME_DECODE_CONST_OFFSET = -180;
constexpr uint INVERSE_INT16_CONST = 32768;
constexpr uint32_t GET_FOURTH_BYTE = 0xFF000000;
constexpr uint32_t GET_THIRD_BYTE = 0x00FF0000;
constexpr uint32_t GET_SECOND_BYTE = 0x0000FF00;
constexpr uint32_t GET_FIRST_BYTE = 0x000000FF;
constexpr uint CARTS_ANGLE_RESOLUTION = 1000;
constexpr float RAD_TO_DEG = (180.0 / M_PI);
constexpr uint8_t CARTS_ENCODER_HIGH_BYTE_POSITION = 1;
constexpr uint8_t CARTS_ENCODER_LOW_BYTE_POSITION = 0;
// the arbitrary zero of the encoders counter on the TTControl, used for easy
// rounding to the beginning of the measures exceeding the 360 degrees
// the encoders natural limits are from 10000 (-360 deg) to 30000(360 deg)
// but practically we should never reach them
constexpr int CARTS_ENCODERS_ZERO_VALUE = 20000;
// carts encoders have 10000 impulses per one full turn (360 deg)
// so the resolution is 360 deg/10000
constexpr float CARTS_ENCODERS_RESOLUTION = 0.036;

constexpr uint DELIMITER_MIDDLE_LENGTH = 1;
constexpr uint DELIMITER_ENDING_LENGTH = 2;

inline uint8_t TTCommControl::extract_fourth_byte(const uint32_t four_byte_number)
{
  return static_cast<uint8_t>((four_byte_number & GET_FOURTH_BYTE) >> 24);
}

inline uint8_t TTCommControl::extract_third_byte(const uint32_t four_byte_number)
{
  return static_cast<uint8_t>((four_byte_number & GET_THIRD_BYTE) >> 16);
}

inline uint8_t TTCommControl::extract_second_byte(const uint32_t four_byte_number)
{
  return static_cast<uint8_t>((four_byte_number & GET_SECOND_BYTE) >> 8);
}

inline uint8_t TTCommControl::extract_first_byte(const uint32_t four_byte_number)
{
  return static_cast<uint8_t>(four_byte_number & GET_FIRST_BYTE);
}

void TTCommControl::prepareVelocitiesFrame(can_frame& frame, const float velocity, const uint8_t start_byte)
{
  int16_t i16_velocity = (velocity * VELO_FRAME_ENCODE_CONST);

  frame.data[start_byte] = extract_second_byte(static_cast<uint32_t>(i16_velocity));
  // if sending velo is below zero
  if (i16_velocity < 0)
  {
    // perform below substraction to achive inversed number
    i16_velocity = INVERSE_INT16_CONST - i16_velocity;
    frame.data[start_byte] = extract_second_byte(static_cast<uint32_t>(i16_velocity));
    // toggle sign bit
    frame.data[start_byte] |= SIGN_BIT;
  }
  else
  {
    frame.data[start_byte] = extract_second_byte(static_cast<uint32_t>(i16_velocity));
  }
  frame.data[start_byte + 1] = extract_first_byte(static_cast<uint32_t>(i16_velocity));
}

float TTCommControl::decodeVelocitiy(const uint8_t start_byte, const can_frame& frame)
{
  return (static_cast<float>((static_cast<int16_t>(frame.data[start_byte] & DATA_BITS)) << 8 |
                             frame.data[start_byte + 1])) *
         VELO_FRAME_DECODE_CONST;
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

float TTCommControl::decodeAngle(const can_frame& frame)
{
  return (static_cast<float>(((static_cast<int32_t>(frame.data[CARTS_ENCODER_HIGH_BYTE_POSITION])) << 8 |
                              (static_cast<int32_t>(frame.data[CARTS_ENCODER_LOW_BYTE_POSITION]))) -
                             CARTS_ENCODERS_ZERO_VALUE) *
          CARTS_ENCODERS_RESOLUTION);
}

uint32_t TTCommControl::decodeEncoderValue(const can_frame& frame)
{
  return ((static_cast<uint32_t>(frame.data[CARTS_ENCODER_HIGH_BYTE_POSITION]) << 8) |
          (static_cast<uint32_t>(frame.data[CARTS_ENCODER_LOW_BYTE_POSITION])));
}

void TTCommControl::setVelocity(const eVelocities enum_velo, const can_frame& frame)
{
  switch (enum_velo)
  {
    case FRONT_LINEAR:
      cart_.setFrontLinear(decodeVelocitiy(FRONT_LINEAR, frame));
      // if there is sign bit present, change sign of velocity
      if (frame.data[FRONT_LINEAR] & SIGN_BIT)
      {
        cart_.setFrontLinear(-cart_.getFrontLinear());
      }
      break;
    case FRONT_ANGULAR:
      cart_.setFrontAngular(decodeVelocitiy(FRONT_ANGULAR, frame) * DEG_TO_RAD);
      if (frame.data[FRONT_ANGULAR] & SIGN_BIT)
      {
        cart_.setFrontAngular(-cart_.getFrontAngular());
      }
      break;
    case BACK_LINEAR:
      cart_.setBackLinear(decodeVelocitiy(BACK_LINEAR, frame));
      if (frame.data[BACK_LINEAR] & SIGN_BIT)
      {
        cart_.setBackLinear(-cart_.getBackLinear());
      }
      break;
    case BACK_ANGULAR:
      cart_.setBackAngular(decodeVelocitiy(BACK_ANGULAR, frame) * DEG_TO_RAD);
      if (frame.data[BACK_ANGULAR] & SIGN_BIT)
      {
        cart_.setBackAngular(-cart_.getBackAngular());
      }
      break;
  }
}

void TTCommControl::setTTControlConnectionStatus(const bool& connected)
{
  if (!connected)
  {
    ttcontrol_carts_angle_confirmed_ = false;
  }
  ttcontrol_is_connected_ = connected;
}

bool TTCommControl::getTTControlConnectionStatus()
{
  return ttcontrol_is_connected_;
}

bool TTCommControl::getEncodersConfirmedStatus() const
{
  return ttcontrol_carts_angle_confirmed_;
}

TTCommControl::TTCommControl(const char* can_channel)
{
  ttcontrol_carts_angle_confirmed_ = false;
  ttcontrol_is_connected_ = false;
  can_handler_.iInitialize(can_channel);
}

void TTCommControl::receiveCanFrames()
{
  can_frame message;
  while (can_handler_.bReceiveFrame(&message))
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

inline bool TTCommControl::checkIfCanBusIdsMatch(const uint32_t id_to_check, const uint msg_id)
{
  return ((id_to_check & ADDR_MASK) == (msg_id & ADDR_MASK));
}

inline void TTCommControl::setReceivedCanBusIdsBitMask(const uint32_t mask_to_set)
{
  received_msgs_register_ |= mask_to_set;
}

inline bool TTCommControl::checkIfBitIsSet(const uint32_t number_to_check, const uint32_t mask_to_check)
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

bool TTCommControl::checkReceivedCanBusIdBit(const uint32_t mask_to_check)
{
  return checkIfBitIsSet(received_msgs_register_, mask_to_check);
}

void TTCommControl::parseCanFrames()
{
  can_frame message;
  while (!frames_queue_.empty())
  {
    message = frames_queue_.front();
    if (checkIfCanBusIdsMatch(VELO_FRAME_ID, message.can_id))
    {
      setVelocity(FRONT_LINEAR, message);
      setVelocity(FRONT_ANGULAR, message);
      setVelocity(BACK_LINEAR, message);
      setVelocity(BACK_ANGULAR, message);
      setReceivedCanBusIdsBitMask(VELOCITIES_MASK);
    }
    if (checkIfCanBusIdsMatch(FRONT_ANGLE_ID, message.can_id))
    {
      front_cart_encoder_unread_ = true;
      front_cart_encoder_value_ = decodeEncoderValue(message);
      front_cart_angle_ = decodeAngle(message) * DEG_TO_RAD;
      setReceivedCanBusIdsBitMask(FRONT_ANGLE_MASK);
    }
    if (checkIfCanBusIdsMatch(BACK_ANGLE_ID, message.can_id))
    {
      back_cart_encoder_unread_ = true;
      back_cart_encoder_value_ = decodeEncoderValue(message);
      back_cart_angle_ = decodeAngle(message) * DEG_TO_RAD;
      setReceivedCanBusIdsBitMask(BACK_ANGLE_MASK);
    }
    if (checkIfCanBusIdsMatch(DELTAS_GEOMETRY_FEEDBACK_ID, message.can_id))
    {
      decodeCartsGeometryState(message);
      setReceivedCanBusIdsBitMask(GEOMETRY_FEEDBACK_MASK);
    }
    if (checkIfCanBusIdsMatch(PROTOCOL_FRAME_ID, message.can_id))
    {
      // if TTControl confirms angle receiving
      if (message.data[PROTOCOL_PURPOSE_BYTE] == TTC_CARTS_ANGLE_INITIALIZATION)
      {
        ttcontrol_carts_angle_confirmed_ = true;
      }
      // if there is a request for entering automatic mode
      if (message.data[PROTOCOL_PURPOSE_BYTE] == TTC_AUTOMATIC_MODE_REQUEST)
      {
        // if TTC said that it is ready for entering automatic mode
        if (message.data[PROTOCOL_DETAIL_BYTE] == TTC_OK)
        {
          setReceivedCanBusIdsBitMask(AUTOMATIC_STATE_REQUEST_MASK);
        }
        // if TTC send some error while entering automatic mode
        else
        {
          err_code_ = message.data[PROTOCOL_DETAIL_BYTE];
          setReceivedCanBusIdsBitMask(AUTOMATIC_STATE_REQUEST_ERROR_MASK);
        }
      }
    }
    frames_queue_.pop();
  }
}

void TTCommControl::confirmAdvantechReadiness()
{
  can_frame message;
  message.can_id = PROTOCOL_RESPONSE_FRAME_ID;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[0] = TTC_AUTOMATIC_MODE_REQUEST;
  message.data[1] = TTCONTROL_J1939_NODE_ID;  // INFO byte, send the TTControl node ID to confirm readiness
  message.data[2] = 0;
  message.data[3] = 0;
  message.data[4] = 0;
  message.data[5] = 0;
  message.data[6] = 0;
  message.data[7] = 0;
  can_handler_.iSendFrame(&message);
}

inline uint8_t TTCommControl::returnDeltasGeometryCommandByte(const uint8_t command)
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
  message.can_id = DELTAS_GEOMETRY_COMMAND_ID;
  message.can_dlc = TWO_BYTE_CAN_FRAME;
  message.data[FRONT_DELTA_GEOMETRY_BYTE] = returnDeltasGeometryCommandByte(front_cart_command);
  message.data[BACK_DELTA_GEOMETRY_BYTE] = returnDeltasGeometryCommandByte(back_cart_command);
  can_handler_.iSendFrame(&message);
}

void TTCommControl::sendCartsEncodersInitialization()
{
  can_frame message;
  message.can_id = PROTOCOL_RESPONSE_FRAME_ID;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[0] = TTC_CARTS_ANGLE_INITIALIZATION;
  message.data[1] = extract_third_byte(init_front_cart_encoder_value_);
  message.data[2] = extract_second_byte(init_front_cart_encoder_value_);
  message.data[3] = extract_first_byte(init_front_cart_encoder_value_);
  message.data[4] = extract_third_byte(init_back_cart_encoder_value_);
  message.data[5] = extract_second_byte(init_back_cart_encoder_value_);
  message.data[6] = extract_first_byte(init_back_cart_encoder_value_);
  message.data[7] = 0;
  can_handler_.iSendFrame(&message);
}

void TTCommControl::sendVelocities(const float front_linear, const float front_angular, const float back_linear,
                                   const float back_angular)
{
  can_frame message;
  message.can_id = VELOCITY_CONTROL_FRAME_ID;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  prepareVelocitiesFrame(message, front_linear, 0);
  prepareVelocitiesFrame(message, front_angular, 2);
  prepareVelocitiesFrame(message, back_linear, 4);
  prepareVelocitiesFrame(message, back_angular, 6);
  can_handler_.iSendFrame(&message);
}

void TTCommControl::sendSignalsParameters(const int ditter_period, const int ditter_amplitude, const int encoder_period)
{
  can_frame message;
  message.can_id = CONFIG_FRAME_CAN_ID;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[CONFIG_FRAME_TYPE_BYTE] = TTControlParamsFrameType.TTCONTROL_SIGNALS_PARAMS_TYPE;
  message.data[DITTER_PERIOD_BYTE] = extract_first_byte(static_cast<uint32_t>(ditter_period));
  message.data[DITTER_AMPLITUDE_FIRST_BYTE] = extract_first_byte(static_cast<uint32_t>(ditter_amplitude));
  message.data[DITTER_AMPLITUDE_FIRST_BYTE + 1] = extract_second_byte(static_cast<uint32_t>(ditter_amplitude));
  message.data[ENCODER_PERIOD_BYTE] = extract_first_byte(static_cast<uint32_t>(encoder_period));
  // dont-care bytes
  message.data[ENCODER_PERIOD_BYTE + 1] = 0xFF;
  message.data[ENCODER_PERIOD_BYTE + 2] = 0xFF;
  message.data[ENCODER_PERIOD_BYTE + 3] = 0xFF;
  can_handler_.iSendFrame(&message);
}

void TTCommControl::sendCartsPIDParameters(const double kp, const double ki, const double kd, const double acc,
                                           const uint8_t frame_type)
{
  can_frame message;
  message.can_id = CONFIG_FRAME_CAN_ID;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[CONFIG_FRAME_TYPE_BYTE] = frame_type;
  message.data[KP_FIRST_BYTE] = extract_first_byte(static_cast<uint32_t>(kp * K_RESOLUTION_MULTIPLIER));
  message.data[KP_FIRST_BYTE + 1] = extract_second_byte(static_cast<uint32_t>(kp * K_RESOLUTION_MULTIPLIER));
  message.data[KI_FIRST_BYTE] = extract_first_byte(static_cast<uint32_t>(ki * K_RESOLUTION_MULTIPLIER));
  message.data[KI_FIRST_BYTE + 1] = extract_second_byte(static_cast<uint32_t>(ki * K_RESOLUTION_MULTIPLIER));
  message.data[KD_FIRST_BYTE] = extract_first_byte(static_cast<uint32_t>(kd * KD_RESOLUTION_MULTIPLIER));
  message.data[ACC_FIRST_BYTE] = extract_first_byte(static_cast<uint32_t>(acc * ACC_RESOLUTION_MULTIPLIER));
  message.data[ACC_FIRST_BYTE + 1] = extract_second_byte(static_cast<uint32_t>(acc * ACC_RESOLUTION_MULTIPLIER));
  can_handler_.iSendFrame(&message);
}

void TTCommControl::sendCaterpillarPIDParameters(const uint kp, const uint ki, const uint kd, const uint8_t frame_type)
{
  can_frame message;
  message.can_id = CONFIG_FRAME_CAN_ID;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[CONFIG_FRAME_TYPE_BYTE] = frame_type;
  message.data[KP_CATERPILLAR_FIRST_BYTE] = extract_first_byte(kp);
  message.data[KP_CATERPILLAR_FIRST_BYTE + 1] = extract_second_byte(kp);
  message.data[KI_CATERPILLAR_FIRST_BYTE] = extract_first_byte(ki);
  message.data[KI_CATERPILLAR_FIRST_BYTE + 1] = extract_second_byte(ki);
  message.data[KD_CATERPILLAR_FIRST_BYTE] = extract_first_byte(kd);
  message.data[KD_CATERPILLAR_FIRST_BYTE + 1] = extract_second_byte(kd);
  // dont care byte
  message.data[KD_CATERPILLAR_FIRST_BYTE + 2] = 0xFF;
  can_handler_.iSendFrame(&message);
}

void TTCommControl::sendCaterpillarBaseSignal(const uint forward_base, const uint backward_base,
                                              const uint8_t frame_type)
{
  can_frame message;
  message.can_id = CONFIG_FRAME_CAN_ID;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[CONFIG_FRAME_TYPE_BYTE] = frame_type;
  message.data[FORWARD_SIGNAL_BASE_FIRST_BYTE] = extract_first_byte(forward_base);
  message.data[FORWARD_SIGNAL_BASE_FIRST_BYTE + 1] = extract_second_byte(forward_base);
  message.data[BACKWARD_SIGNAL_BASE_FIRST_BYTE] = extract_first_byte(backward_base);
  message.data[BACKWARD_SIGNAL_BASE_FIRST_BYTE + 1] = extract_second_byte(backward_base);
  // dont care bytes
  message.data[BACKWARD_SIGNAL_BASE_FIRST_BYTE + 2] = 0xFF;
  message.data[BACKWARD_SIGNAL_BASE_FIRST_BYTE + 3] = 0xFF;
  message.data[BACKWARD_SIGNAL_BASE_FIRST_BYTE + 4] = 0xFF;
  can_handler_.iSendFrame(&message);
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

int TTCommControl::getErrorCode() const
{
  return err_code_;
}

void TTCommControl::setPathToCartsEncodersDataFile(const std::string& path_to_file)
{
  path_to_encoder_file_ = path_to_file;
}

uint32_t TTCommControl::extractEncoderInitValueFromFile(std::ifstream& file_input)
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
