/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <hal_ttcontrol_communication/TTCommControl.h>

// mask used for extracting address and source, without PG
constexpr uint ADDR_MASK = 0xFFFF;
constexpr uint8_t FULL_CAN_FRAME_BYTE_NUMBER = 8;
constexpr uint PROTOCOL_FRAME_ID = 0x00EFC864;
constexpr uint PROTOCOL_RESPONSE_FRAME_ID = 0x00EF64C8;
constexpr uint VELO_FRAME_ID = 0x00FFD064;
constexpr uint FRONT_ANGLE_ID = 0xAA20;
constexpr uint BACK_ANGLE_ID = 0xAA21;
constexpr uint VELOCITY_CONTROL_FRAME_ID = 0x00FFD0C8;
constexpr uint CONFIG_FRAME_CAN_ID = 0x00FFDBC8;
constexpr double DEG_TO_RAD = (M_PI / 180.0);

constexpr uint SIGN_BIT = 0x80;
constexpr uint DATA_BITS = 0x7F;

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
      _cart.vSetFrontLinear(fDecodeVelocitiy(FRONT_LINEAR, frame));
      // if there is sign bit present, change sign of velocity
      if (frame.data[FRONT_LINEAR] & SIGN_BIT)
      {
        _cart.vSetFrontLinear(-_cart.fGetFrontLinear());
      }
      break;
    case FRONT_ANGULAR:
      _cart.vSetFrontAngular(fDecodeVelocitiy(FRONT_ANGULAR, frame) * DEG_TO_RAD);
      if (frame.data[FRONT_ANGULAR] & SIGN_BIT)
      {
        _cart.vSetFrontAngular(-_cart.fGetFrontAngular());
      }
      break;
    case BACK_LINEAR:
      _cart.vSetBackLinear(fDecodeVelocitiy(BACK_LINEAR, frame));
      if (frame.data[BACK_LINEAR] & SIGN_BIT)
      {
        _cart.vSetBackLinear(-_cart.fGetBackLinear());
      }
      break;
    case BACK_ANGULAR:
      _cart.vSetBackAngular(fDecodeVelocitiy(BACK_ANGULAR, frame) * DEG_TO_RAD);
      if (frame.data[BACK_ANGULAR] & SIGN_BIT)
      {
        _cart.vSetBackAngular(-_cart.fGetBackAngular());
      }
      break;
  }
}

void TTCommControl::vSetTTControlConnectionStatus(const bool& connected)
{
  if (!connected)
  {
    _ttcontrol_carts_angle_confirmed = false;
  }
  else
  {
    _ttcontrol_is_connected = connected;
  }
}

bool TTCommControl::bGetTTControlConnectionStatus()
{
  return _ttcontrol_is_connected;
}

bool TTCommControl::bGetEncodersConfirmedStatus()
{
  return _ttcontrol_carts_angle_confirmed;
}

TTCommControl::TTCommControl(const char* can_channel)
{
  _ttcontrol_carts_angle_confirmed = false;
  _ttcontrol_is_connected = false;
  _can_handler.iInitialize(can_channel);
}

void TTCommControl::vReceiveCanFrames()
{
  can_frame message;
  while (_can_handler.bReceiveFrame(&message))
  {
    _frames_queue.push(message);
  }
}

TTCommControl::~TTCommControl()
{
  while (!_frames_queue.empty())
  {
    _frames_queue.pop();
  }
}

uint8_t TTCommControl::u8ParseCanFrames()
{
  uint8_t status = 0;
  can_frame message;
  while (!_frames_queue.empty())
  {
    message = _frames_queue.front();
    // if there is velocities frame in _frames_queue
    if ((VELO_FRAME_ID & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      vSetVelocitiy(FRONT_LINEAR, message);
      vSetVelocitiy(FRONT_ANGULAR, message);
      vSetVelocitiy(BACK_LINEAR, message);
      vSetVelocitiy(BACK_ANGULAR, message);
      status |= VELOCITIES_MASK;
    }
    // if there is front cart angle frame in _frames_queue
    if ((FRONT_ANGLE_ID & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      _front_cart_encoder_unread = true;
      _front_cart_encoder_value = uiDecodeEncoderValue(message);
      _front_cart_angle = fDecodeAngle(message) * DEG_TO_RAD;
      status |= FRONT_ANGLE_MASK;
    }
    // if there is back cart angle frame in _frames_queue
    if ((BACK_ANGLE_ID & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      _back_cart_encoder_unread = true;
      _back_cart_encoder_value = uiDecodeEncoderValue(message);
      _back_cart_angle = fDecodeAngle(message) * DEG_TO_RAD;
      status |= BACK_ANGLE_MASK;
    }
    // if there is protocol frame in _frames_queue
    if ((PROTOCOL_FRAME_ID & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      // if TTControl confirms angle receiving
      if (message.data[0] == TTC_CARTS_ANGLE_INITIALIZATION)
      {
        _ttcontrol_carts_angle_confirmed = true;
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
          _err_code = message.data[1];
          status |= AUTOMATIC_STATE_REQUEST_ERROR_MASK;
        }
      }
    }
    _frames_queue.pop();
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
  _can_handler.iSendFrame(&message);
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
  message.data[1] = static_cast<uint8_t>((_init_front_cart_encoder_value & GET_THIRD_BYTE) >> 16);
  message.data[2] = static_cast<uint8_t>((_init_front_cart_encoder_value & GET_SECOND_BYTE) >> 8);
  message.data[3] = static_cast<uint8_t>((_init_front_cart_encoder_value & GET_FIRST_BYTE));
  message.data[4] = static_cast<uint8_t>((_init_back_cart_encoder_value & GET_THIRD_BYTE) >> 16);
  message.data[5] = static_cast<uint8_t>((_init_back_cart_encoder_value & GET_SECOND_BYTE) >> 8);
  message.data[6] = static_cast<uint8_t>((_init_back_cart_encoder_value & GET_FIRST_BYTE));
  message.data[7] = 0;
  _can_handler.iSendFrame(&message);
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
  _can_handler.iSendFrame(&message);
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
  _can_handler.iSendFrame(&message);
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
  _can_handler.iSendFrame(&message);
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
  _can_handler.iSendFrame(&message);
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
  _can_handler.iSendFrame(&message);
}

CartsStatus TTCommControl::getCartsStatus()
{
  return _cart;
}

float TTCommControl::fGetFrontCartAngle()
{
  return _front_cart_angle;
}

float TTCommControl::fGetBackCartAngle()
{
  return _back_cart_angle;
}

int TTCommControl::iGetErrorCode()
{
  return _err_code;
}

void TTCommControl::vSetPathToCartsEncodersDataFile(const std::string& path_to_file)
{
  _path_to_encoder_file = path_to_file;
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
    encoder_file_name = _file_name;
  }
  else
  {
    encoder_file_name = _file_name_backup;
  }
  _encoder_data_file_input.open(_path_to_encoder_file + encoder_file_name);
  if (!_encoder_data_file_input.is_open() || _encoder_data_file_input.fail())
  {
    _encoder_data_file_input.close();
    return false;
  }
  // first line for the front cart encoder value
  _init_front_cart_encoder_value = iExtractEncoderInitValueFromFile(_encoder_data_file_input);
  if (_init_front_cart_encoder_value == 0)
  {
    _encoder_data_file_input.close();
    return false;
  }

  // second line for the back cart encoder value
  _init_back_cart_encoder_value = iExtractEncoderInitValueFromFile(_encoder_data_file_input);
  if (_init_back_cart_encoder_value == 0)
  {
    _encoder_data_file_input.close();
    return false;
  }

  _encoder_data_file_input.close();
  return true;
}

bool TTCommControl::bWriteEncodersDataFile()
{
  // check if angle data is complete
  if (_front_cart_encoder_unread && _back_cart_encoder_unread)
  {
    // save the proper file
    _encoder_data_file_output.open((_path_to_encoder_file + _file_name), std::ios::trunc);
    if (!_encoder_data_file_output.is_open())
    {
      _encoder_data_file_output.close();
      return false;
    }
    _encoder_data_file_output << "front:" << _front_cart_encoder_value << ";\n";
    _encoder_data_file_output << "back:" << _back_cart_encoder_value << ";";
    _encoder_data_file_output.close();

    // save the backup file
    _encoder_data_file_output.open((_path_to_encoder_file + _file_name_backup), std::ios::trunc);
    if (!_encoder_data_file_output.is_open())
    {
      _encoder_data_file_output.close();
      return false;
    }
    _encoder_data_file_output << "front:" << _front_cart_encoder_value << ";\n";
    _encoder_data_file_output << "back:" << _back_cart_encoder_value << ";";
    _encoder_data_file_output.close();
    // reset flags
    _front_cart_encoder_unread = false;
    _back_cart_encoder_unread = false;
  }

  return true;
}
