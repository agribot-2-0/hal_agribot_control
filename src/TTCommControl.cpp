/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <hal_ttcontrol_communication/TTCommControl.h>

// mask used for extracting address and source, without PG
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
constexpr uint32_t GET_THIRD_BYTE = 0xFF0000;
constexpr uint32_t GET_SECOND_BYTE = 0x00FF00;
constexpr uint32_t GET_FIRST_BYTE = 0x0000FF;
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

void TTCommControl::vPrepareVelocitiesFrame(can_frame* frame, float velocity, uint8_t start_byte)
{
  int16_t i16_velocity = (VELO_FRAME_ENCODE_CONST * velocity);

  frame->data[start_byte] = (uint8_t)((i16_velocity & GET_SECOND_BYTE) >> 8);
  // if sending velo is below zero
  if (i16_velocity < 0)
  {
    // perform below substraction to achive inversed number
    i16_velocity = INVERSE_INT16_CONST - i16_velocity;
    frame->data[start_byte] = (uint8_t)((i16_velocity & GET_SECOND_BYTE) >> 8);
    // toggle sign bit
    frame->data[start_byte] |= SIGN_BIT;
  }
  frame->data[start_byte + 1] = (uint8_t)i16_velocity;
}

float TTCommControl::fDecodeVelocitiy(uint8_t start_byte, can_frame frame)
{
  return (static_cast<float>((static_cast<int16_t>(frame.data[start_byte] & DATA_BITS)) << 8 |
                             frame.data[start_byte + 1])) *
         VELO_FRAME_DECODE_CONST;
}

void TTCommControl::vDecodeCartsGeometryState(const can_frame& frame)
{
  if (frame.data[FRONT_DELTA_GEOMETRY_BYTE] == DELTAS_GEOMETRY_STATE_CHANGED)
  {
    cart_.vSetFrontCartGeometryState(DELTAS_GEOMETRY_STATE_CHANGED);
  }
  else if (frame.data[FRONT_DELTA_GEOMETRY_BYTE] == DELTAS_GEOMETRY_STATE_NORMAL)
  {
    cart_.vSetFrontCartGeometryState(DELTAS_GEOMETRY_STATE_NORMAL);
  }
  else
  {
    cart_.vSetFrontCartGeometryState(DELTAS_GEOMETRY_STATE_UNKNOWN);
  }

  if (frame.data[BACK_DELTA_GEOMETRY_BYTE] == DELTAS_GEOMETRY_STATE_CHANGED)
  {
    cart_.vSetBackCartGeometryState(DELTAS_GEOMETRY_STATE_CHANGED);
  }
  else if (frame.data[BACK_DELTA_GEOMETRY_BYTE] == DELTAS_GEOMETRY_STATE_NORMAL)
  {
    cart_.vSetBackCartGeometryState(DELTAS_GEOMETRY_STATE_NORMAL);
  }
  else
  {
    cart_.vSetBackCartGeometryState(DELTAS_GEOMETRY_STATE_UNKNOWN);
  }
}

float TTCommControl::fDecodeAngle(can_frame frame)
{
  return (static_cast<float>(((static_cast<int32_t>(frame.data[CARTS_ENCODER_HIGH_BYTE_POSITION])) << 8 |
                              (static_cast<int32_t>(frame.data[CARTS_ENCODER_LOW_BYTE_POSITION]))) -
                             CARTS_ENCODERS_ZERO_VALUE) *
          CARTS_ENCODERS_RESOLUTION);
}

uint TTCommControl::uiDecodeEncoderValue(can_frame frame)
{
  return ((static_cast<uint32_t>(frame.data[CARTS_ENCODER_HIGH_BYTE_POSITION]) << 8) |
          (static_cast<uint32_t>(frame.data[CARTS_ENCODER_LOW_BYTE_POSITION])));
}

void TTCommControl::vSetVelocitiy(eVelocities enum_velo, can_frame frame)
{
  switch (enum_velo)
  {
    case FRONT_LINEAR:
      cart_.vSetFrontLinear(fDecodeVelocitiy(FRONT_LINEAR, frame));
      // if there is sign bit present, change sign of velocity
      if (frame.data[FRONT_LINEAR] & SIGN_BIT)
      {
        cart_.vSetFrontLinear(-cart_.fGetFrontLinear());
      }
      break;
    case FRONT_ANGULAR:
      cart_.vSetFrontAngular(fDecodeVelocitiy(FRONT_ANGULAR, frame) * DEG_TO_RAD);
      if (frame.data[FRONT_ANGULAR] & SIGN_BIT)
      {
        cart_.vSetFrontAngular(-cart_.fGetFrontAngular());
      }
      break;
    case BACK_LINEAR:
      cart_.vSetBackLinear(fDecodeVelocitiy(BACK_LINEAR, frame));
      if (frame.data[BACK_LINEAR] & SIGN_BIT)
      {
        cart_.vSetBackLinear(-cart_.fGetBackLinear());
      }
      break;
    case BACK_ANGULAR:
      cart_.vSetBackAngular(fDecodeVelocitiy(BACK_ANGULAR, frame) * DEG_TO_RAD);
      if (frame.data[BACK_ANGULAR] & SIGN_BIT)
      {
        cart_.vSetBackAngular(-cart_.fGetBackAngular());
      }
      break;
  }
}

void TTCommControl::vSetTTControlConnectionStatus(const bool& connected)
{
  if (!connected)
  {
    ttcontrol_carts_angle_confirmed_ = false;
  }
  else
  {
    ttcontrol_is_connected_ = connected;
  }
}

bool TTCommControl::bGetTTControlConnectionStatus()
{
  return ttcontrol_is_connected_;
}

bool TTCommControl::bGetEncodersConfirmedStatus()
{
  return ttcontrol_carts_angle_confirmed_;
}

TTCommControl::TTCommControl(const char* can_channel)
{
  ttcontrol_carts_angle_confirmed_ = false;
  ttcontrol_is_connected_ = false;
  can_handler_.iInitialize(can_channel);
}

void TTCommControl::vReceiveCanFrames()
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

uint8_t TTCommControl::u8ParseCanFrames()
{
  uint8_t status = 0;
  can_frame message;
  while (!frames_queue_.empty())
  {
    message = frames_queue_.front();
    // if there is velocities frame in frames_queue_
    if ((VELO_FRAME_ID & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      vSetVelocitiy(FRONT_LINEAR, message);
      vSetVelocitiy(FRONT_ANGULAR, message);
      vSetVelocitiy(BACK_LINEAR, message);
      vSetVelocitiy(BACK_ANGULAR, message);
      status |= VELOCITIES_MASK;
    }
    // if there is front cart angle frame in frames_queue_
    if ((FRONT_ANGLE_ID & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      front_cart_encoder_unread_ = true;
      front_cart_encoder_value_ = uiDecodeEncoderValue(message);
      front_cart_angle_ = fDecodeAngle(message) * DEG_TO_RAD;
      status |= FRONT_ANGLE_MASK;
    }
    // if there is back cart angle frame in frames_queue_
    if ((BACK_ANGLE_ID & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      back_cart_encoder_unread_ = true;
      back_cart_encoder_value_ = uiDecodeEncoderValue(message);
      back_cart_angle_ = fDecodeAngle(message) * DEG_TO_RAD;
      status |= BACK_ANGLE_MASK;
    }
    // if there is back cart angle frame in frames_queue_
    if ((BACK_ANGLE_ID & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      back_cart_encoder_unread_ = true;
      back_cart_encoder_value_ = uiDecodeEncoderValue(message);
      back_cart_angle_ = fDecodeAngle(message) * DEG_TO_RAD;
      status |= BACK_ANGLE_MASK;
    }
    // if there is a cart geometry feedback
    if ((DELTAS_GEOMETRY_FEEDBACK_ID & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      vDecodeCartsGeometryState(message);
      status |= GEOMETRY_FEEDBACK_MASK;
    }
    // if there is protocol frame in frames_queue_
    if ((PROTOCOL_FRAME_ID & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      // if TTControl confirms angle receiving
      if (message.data[0] == TTC_CARTS_ANGLE_INITIALIZATION)
      {
        ttcontrol_carts_angle_confirmed_ = true;
      }
      // if there is a request for entering automatic mode
      if (message.data[0] == TTC_AUTOMATIC_MODE_REQUEST)
      {
        // if TTC said that it is ready for entering automatic mode
        if (message.data[1] == TTC_OK)
        {
          status |= AUTOMATIC_STATE_REQUEST_MASK;
        }
        // if TTC send some error while entering automatic mode
        else
        {
          err_code_ = message.data[1];
          status |= AUTOMATIC_STATE_REQUEST_ERROR_MASK;
        }
      }
    }
    frames_queue_.pop();
  }
  return status;
}

void TTCommControl::vConfirmAdvantechReadiness()
{
  can_frame message;
  message.can_id = PROTOCOL_RESPONSE_FRAME_ID;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[0] = TTC_AUTOMATIC_MODE_REQUEST;
  message.data[1] = 0x64;  // INFO byte (0x64 - ok)
  message.data[2] = 0;
  message.data[3] = 0;
  message.data[4] = 0;
  message.data[5] = 0;
  message.data[6] = 0;
  message.data[7] = 0;
  can_handler_.iSendFrame(&message);
}

void TTCommControl::vSendDeltasGeometryCommand(const uint8_t& front_cart_command, const uint8_t& back_cart_command)
{
  can_frame message;
  message.can_id = DELTAS_GEOMETRY_COMMAND_ID;
  message.can_dlc = TWO_BYTE_CAN_FRAME;
  switch (front_cart_command)
  {
    case DELTAS_GEOMETRY_STATE_CHANGED:
      message.data[FRONT_DELTA_GEOMETRY_BYTE] = DELTAS_GEOMETRY_STATE_CHANGED;
      break;
    case DELTAS_GEOMETRY_STATE_NORMAL:
      message.data[FRONT_DELTA_GEOMETRY_BYTE] = DELTAS_GEOMETRY_STATE_NORMAL;
      break;
    default:
      message.data[FRONT_DELTA_GEOMETRY_BYTE] = DELTAS_GEOMETRY_STATE_UNKNOWN;
      break;
  }
  switch (back_cart_command)
  {
    case DELTAS_GEOMETRY_STATE_CHANGED:
      message.data[BACK_DELTA_GEOMETRY_BYTE] = DELTAS_GEOMETRY_STATE_CHANGED;
      break;
    case DELTAS_GEOMETRY_STATE_NORMAL:
      message.data[BACK_DELTA_GEOMETRY_BYTE] = DELTAS_GEOMETRY_STATE_NORMAL;
      break;
    default:
      message.data[BACK_DELTA_GEOMETRY_BYTE] = DELTAS_GEOMETRY_STATE_UNKNOWN;
      break;
  }
  can_handler_.iSendFrame(&message);
}

void TTCommControl::vSendCartsEncodersInitialization()
{
  // uint front_data_to_send = static_cast<int>(_fInitFrontCartAngle * RAD_TO_DEG + CARTS_ANGLES_OFFSET) *
  // CARTS_ANGLE_RESOLUTION; uint back_data_to_send = static_cast<int>(_fInitBackCartAngle * RAD_TO_DEG +
  // CARTS_ANGLES_OFFSET) * CARTS_ANGLE_RESOLUTION;
  can_frame message;
  message.can_id = PROTOCOL_RESPONSE_FRAME_ID;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[0] = TTC_CARTS_ANGLE_INITIALIZATION;
  message.data[1] = static_cast<uint8_t>((init_front_cart_encoder_value_ & GET_THIRD_BYTE) >> 16);
  message.data[2] = static_cast<uint8_t>((init_front_cart_encoder_value_ & GET_SECOND_BYTE) >> 8);
  message.data[3] = static_cast<uint8_t>((init_front_cart_encoder_value_ & GET_FIRST_BYTE));
  message.data[4] = static_cast<uint8_t>((init_back_cart_encoder_value_ & GET_THIRD_BYTE) >> 16);
  message.data[5] = static_cast<uint8_t>((init_back_cart_encoder_value_ & GET_SECOND_BYTE) >> 8);
  message.data[6] = static_cast<uint8_t>((init_back_cart_encoder_value_ & GET_FIRST_BYTE));
  message.data[7] = 0;
  can_handler_.iSendFrame(&message);
}

void TTCommControl::vSendVelocities(const float& front_linear, const float& front_angular, const float& back_linear,
                                    const float& back_angular)
{
  can_frame message;
  message.can_id = VELOCITY_CONTROL_FRAME_ID;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  vPrepareVelocitiesFrame(&message, front_linear, 0);
  vPrepareVelocitiesFrame(&message, front_angular, 2);
  vPrepareVelocitiesFrame(&message, back_linear, 4);
  vPrepareVelocitiesFrame(&message, back_angular, 6);
  can_handler_.iSendFrame(&message);
}

void TTCommControl::vSendSignalsParameters(const int& ditter_period, const int& ditter_amplitude,
                                           const int& encoder_period)
{
  can_frame message;
  message.can_id = CONFIG_FRAME_CAN_ID;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[CONFIG_FRAME_TYPE_BYTE] = TTControlParamsFrameType.TTCONTROL_SIGNALS_PARAMS_TYPE;
  message.data[DITTER_PERIOD_BYTE] = static_cast<uint8_t>(ditter_period);
  message.data[DITTER_AMPLITUDE_FIRST_BYTE] = static_cast<uint8_t>(ditter_amplitude & GET_FIRST_BYTE);
  message.data[DITTER_AMPLITUDE_FIRST_BYTE + 1] = static_cast<uint8_t>((ditter_amplitude & GET_SECOND_BYTE) >> 8);
  message.data[ENCODER_PERIOD_BYTE] = static_cast<uint8_t>(encoder_period);
  // dont-care bytes
  message.data[ENCODER_PERIOD_BYTE + 1] = 0xFF;
  message.data[ENCODER_PERIOD_BYTE + 2] = 0xFF;
  message.data[ENCODER_PERIOD_BYTE + 3] = 0xFF;
  can_handler_.iSendFrame(&message);
}

void TTCommControl::vSendCartsPIDParameters(const double& kp, const double& ki, const double& kd, const double& acc,
                                            const uint8_t& frame_type)
{
  can_frame message;
  message.can_id = CONFIG_FRAME_CAN_ID;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[CONFIG_FRAME_TYPE_BYTE] = frame_type;
  message.data[KP_FIRST_BYTE] = static_cast<uint8_t>(static_cast<uint>(kp * K_RESOLUTION_MULTIPLIER) & GET_FIRST_BYTE);
  message.data[KP_FIRST_BYTE + 1] =
      static_cast<uint8_t>((static_cast<uint>(kp * K_RESOLUTION_MULTIPLIER) & GET_SECOND_BYTE) >> 8);
  message.data[KI_FIRST_BYTE] = static_cast<uint8_t>(static_cast<uint>(ki * K_RESOLUTION_MULTIPLIER) & GET_FIRST_BYTE);
  message.data[KI_FIRST_BYTE + 1] =
      static_cast<uint8_t>((static_cast<uint>(ki * K_RESOLUTION_MULTIPLIER) & GET_SECOND_BYTE) >> 8);
  message.data[KD_FIRST_BYTE] = static_cast<uint8_t>(static_cast<uint>(kd * KD_RESOLUTION_MULTIPLIER) & GET_FIRST_BYTE);
  message.data[ACC_FIRST_BYTE] =
      static_cast<uint8_t>(static_cast<uint>(acc * ACC_RESOLUTION_MULTIPLIER) & GET_FIRST_BYTE);
  message.data[ACC_FIRST_BYTE + 1] =
      static_cast<uint8_t>((static_cast<uint>(acc * ACC_RESOLUTION_MULTIPLIER) & GET_SECOND_BYTE) >> 8);
  can_handler_.iSendFrame(&message);
}

void TTCommControl::vSendCaterpillarPIDParameters(const uint& kp, const uint& ki, const uint& kd,
                                                  const uint8_t& frame_type)
{
  can_frame message;
  message.can_id = CONFIG_FRAME_CAN_ID;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[CONFIG_FRAME_TYPE_BYTE] = frame_type;
  message.data[KP_CATERPILLAR_FIRST_BYTE] = static_cast<uint8_t>(kp & GET_FIRST_BYTE);
  message.data[KP_CATERPILLAR_FIRST_BYTE + 1] = static_cast<uint8_t>((kp & GET_SECOND_BYTE) >> 8);
  message.data[KI_CATERPILLAR_FIRST_BYTE] = static_cast<uint8_t>(ki & GET_FIRST_BYTE);
  message.data[KI_CATERPILLAR_FIRST_BYTE + 1] = static_cast<uint8_t>((ki & GET_SECOND_BYTE) >> 8);
  message.data[KD_CATERPILLAR_FIRST_BYTE] = static_cast<uint8_t>(kd & GET_FIRST_BYTE);
  message.data[KD_CATERPILLAR_FIRST_BYTE + 1] = static_cast<uint8_t>((kd & GET_SECOND_BYTE) >> 8);
  // dont care byte
  message.data[KD_CATERPILLAR_FIRST_BYTE + 2] = 0xFF;
  can_handler_.iSendFrame(&message);
}

void TTCommControl::vSendCaterpillarBaseSignal(const uint& forward_base, const uint& backward_base,
                                               const uint8_t& frame_type)
{
  can_frame message;
  message.can_id = CONFIG_FRAME_CAN_ID;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[CONFIG_FRAME_TYPE_BYTE] = frame_type;
  message.data[FORWARD_SIGNAL_BASE_FIRST_BYTE] = static_cast<uint8_t>(forward_base & GET_FIRST_BYTE);
  message.data[FORWARD_SIGNAL_BASE_FIRST_BYTE + 1] = static_cast<uint8_t>((forward_base & GET_SECOND_BYTE) >> 8);
  message.data[BACKWARD_SIGNAL_BASE_FIRST_BYTE] = static_cast<uint8_t>(backward_base & GET_FIRST_BYTE);
  message.data[BACKWARD_SIGNAL_BASE_FIRST_BYTE + 1] = static_cast<uint8_t>((backward_base & GET_SECOND_BYTE) >> 8);
  // dont care bytes
  message.data[BACKWARD_SIGNAL_BASE_FIRST_BYTE + 2] = 0xFF;
  message.data[BACKWARD_SIGNAL_BASE_FIRST_BYTE + 3] = 0xFF;
  message.data[BACKWARD_SIGNAL_BASE_FIRST_BYTE + 4] = 0xFF;
  can_handler_.iSendFrame(&message);
}

CartsStatus TTCommControl::getCartsStatus()
{
  return cart_;
}

float TTCommControl::fGetFrontCartAngle()
{
  return front_cart_angle_;
}

float TTCommControl::fGetBackCartAngle()
{
  return back_cart_angle_;
}

int TTCommControl::iGetErrorCode()
{
  return err_code_;
}

void TTCommControl::vSetPathToCartsEncodersDataFile(const std::string& path_to_file)
{
  path_to_encoder_file_ = path_to_file;
}

uint TTCommControl::iExtractEncoderInitValueFromFile(std::ifstream& file_input)
{
  const char delimiter_middle = ':';
  const char delimiter_endline = ';';
  std::string line;
  std::string number_str;
  std::getline(file_input, line);
  if (line.size() > 0)
  {
    // check if middle delimiter is in the string, without it, the substr function
    // can return exception because find function return position greater than the str length when
    // the char is not found
    if (line.find(delimiter_middle) > line.length())
    {
      return 0;
    }
    number_str = line.substr(line.find(delimiter_middle));
    if (number_str.front() == delimiter_middle && number_str.back() == delimiter_endline)
    {
      number_str = number_str.substr(DELIMITER_MIDDLE_LENGTH, number_str.length() - DELIMITER_ENDING_LENGTH);
      return std::stoi(number_str);
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

bool TTCommControl::bReadCartsEncodersDataFile(const bool& regular_file)
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
  init_front_cart_encoder_value_ = iExtractEncoderInitValueFromFile(encoder_data_file_input_);
  if (init_front_cart_encoder_value_ == 0)
  {
    encoder_data_file_input_.close();
    return false;
  }

  // second line for the back cart encoder value
  init_back_cart_encoder_value_ = iExtractEncoderInitValueFromFile(encoder_data_file_input_);
  if (init_back_cart_encoder_value_ == 0)
  {
    encoder_data_file_input_.close();
    return false;
  }

  encoder_data_file_input_.close();
  return true;
}

bool TTCommControl::bWriteEncodersDataFile()
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
