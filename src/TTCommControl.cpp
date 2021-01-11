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
constexpr double DEG_TO_RAD = (M_PI / 180);

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
constexpr uint GET_SECOND_BYTE = 0xFF00;
constexpr uint GET_FIRST_BYTE = 0x00FF;

void TTCommControl::vPrepareVelocitiesFrame(can_frame* Frame, float fVelocity, uint8_t u8StartBit)
{
  int16_t i16Velocity = (VELO_FRAME_ENCODE_CONST * fVelocity);

  (*Frame).data[u8StartBit] = (uint8_t)((i16Velocity & GET_SECOND_BYTE) >> 8);
  // if sending velo is below zero
  if (i16Velocity < 0)
  {
    // perform below substraction to achive inversed number
    i16Velocity = INVERSE_INT16_CONST - i16Velocity;
    (*Frame).data[u8StartBit] = (uint8_t)((i16Velocity & GET_SECOND_BYTE) >> 8);
    // toggle sign bit
    (*Frame).data[u8StartBit] |= SIGN_BIT;
  }
  (*Frame).data[u8StartBit + 1] = (uint8_t)i16Velocity;
}

float TTCommControl::fDecodeVelocitiy(uint8_t u8StartBit, can_frame Frame)
{
  return (static_cast<float>(((uint16_t)(Frame.data[u8StartBit] & DATA_BITS)) << 8 | Frame.data[u8StartBit + 1])) *
         VELO_FRAME_DECODE_CONST;
}

float TTCommControl::fDecodeAngle(can_frame Frame)
{
  return (static_cast<float>(((int32_t)Frame.data[2]) << 16 | ((int32_t)Frame.data[1]) << 8 |
                             ((int32_t)Frame.data[0]))) *
             ANGLE_FRAME_DECODE_CONST_MULTIPLIER +
         ANGLE_FRAME_DECODE_CONST_OFFSET;
}

void TTCommControl::vSetVelocitiy(eVelocities eVelo, can_frame Frame)
{
  switch (eVelo)
  {
    case FRONT_LINEAR:
      _Cart.vSetFrontLinear(fDecodeVelocitiy(FRONT_LINEAR, Frame));
      // if there is sign bit present, change sign of velocity
      if (Frame.data[FRONT_LINEAR] & SIGN_BIT)
      {
        _Cart.vSetFrontLinear(-_Cart.fGetFrontLinear());
      }
      break;
    case FRONT_ANGULAR:
      _Cart.vSetFrontAngular(fDecodeVelocitiy(FRONT_ANGULAR, Frame) * DEG_TO_RAD);
      if (Frame.data[FRONT_ANGULAR] & SIGN_BIT)
      {
        _Cart.vSetFrontAngular(-_Cart.fGetFrontAngular());
      }
      break;
    case BACK_LINEAR:
      _Cart.vSetBackLinear(fDecodeVelocitiy(BACK_LINEAR, Frame));
      if (Frame.data[BACK_LINEAR] & SIGN_BIT)
      {
        _Cart.vSetBackLinear(-_Cart.fGetBackLinear());
      }
      break;
    case BACK_ANGULAR:
      _Cart.vSetBackAngular(fDecodeVelocitiy(BACK_ANGULAR, Frame) * DEG_TO_RAD);
      if (Frame.data[BACK_ANGULAR] & SIGN_BIT)
      {
        _Cart.vSetBackAngular(-_Cart.fGetBackAngular());
      }
      break;
  }
}

TTCommControl::TTCommControl(const char* pchCanChannel)
{
  _CanHandler.iInitialize(pchCanChannel);
}

void TTCommControl::vReceiveCanFrames()
{
  can_frame message;
  while (_CanHandler.bReceiveFrame(&message))
  {
    _FramesQueue.push(message);
  }
}

TTCommControl::~TTCommControl()
{
  while (!_FramesQueue.empty())
  {
    _FramesQueue.pop();
  }
}

uint8_t TTCommControl::u8ParseCanFrames()
{
  uint8_t u8Status = 0;
  can_frame message;
  while (!_FramesQueue.empty())
  {
    message = _FramesQueue.front();
    // if there is velocities frame in _FramesQueue
    if ((VELO_FRAME_ID & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      vSetVelocitiy(FRONT_LINEAR, message);
      vSetVelocitiy(FRONT_ANGULAR, message);
      vSetVelocitiy(BACK_LINEAR, message);
      vSetVelocitiy(BACK_ANGULAR, message);
      u8Status |= VELOCITIES_MASK;
    }
    // if there is front cart angle frame in _FramesQueue
    if ((FRONT_ANGLE_ID & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      _fFrontCartAngle = fDecodeAngle(message) * DEG_TO_RAD;
      u8Status |= FRONT_ANGLE_MASK;
    }
    // if there is back cart angle frame in _FramesQueue
    if ((BACK_ANGLE_ID & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      _fBackCartAngle = fDecodeAngle(message) * DEG_TO_RAD;
      u8Status |= BACK_ANGLE_MASK;
    }
    // if there is protocol frame in _FramesQueue
    if ((PROTOCOL_FRAME_ID & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      // if there is a request for entering automatic mode
      if (message.data[0] == TTC_AUTOMATIC_MODE_REQUEST)
      {
        // if TTC said that it is ready for entering automatic mode
        if (message.data[1] == TTC_OK)
        {
          u8Status |= AUTOMATIC_STATE_REQUEST_MASK;
        }
        // if TTC send some error while entering automatic mode
        else
        {
          _iErrCode = message.data[1];
          u8Status |= AUTOMATIC_STATE_REQUEST_ERROR_MASK;
        }
      }
    }
    _FramesQueue.pop();
  }
  return u8Status;
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
  _CanHandler.iSendFrame(&message);
}

void TTCommControl::vSendVelocities(float fFrontLinear, float fFrontAngular, float fBackLinear, float fBackAngular)
{
  can_frame message;
  message.can_id = VELOCITY_CONTROL_FRAME_ID;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  vPrepareVelocitiesFrame(&message, fFrontLinear, 0);
  vPrepareVelocitiesFrame(&message, fFrontAngular, 2);
  vPrepareVelocitiesFrame(&message, fBackLinear, 4);
  vPrepareVelocitiesFrame(&message, fBackAngular, 6);
  _CanHandler.iSendFrame(&message);
}

void TTCommControl::vSendSignalsParameters(int ditter_period, int ditter_amplitude, int encoder_period)
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
  _CanHandler.iSendFrame(&message);
}

void TTCommControl::vSendCartsPIDParameters(double kp, double ki, double kd, double acc, uint8_t frame_type)
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
  _CanHandler.iSendFrame(&message);
}

void TTCommControl::vSendCaterpillarPIDParameters(uint kp, uint ki, uint kd, uint8_t frame_type)
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
  _CanHandler.iSendFrame(&message);
}

void TTCommControl::vSendCaterpillarBaseSignal(uint forward_base, uint backward_base, uint8_t frame_type)
{
  can_frame message;
  message.can_id = CONFIG_FRAME_CAN_ID;
  message.can_dlc = FULL_CAN_FRAME_BYTE_NUMBER;
  message.data[CONFIG_FRAME_TYPE_BYTE] = frame_type;
  message.data[FORWARD_SIGNAL_BASE_FIRST_BYTE] =
      static_cast<uint8_t>(forward_base & GET_FIRST_BYTE);
  message.data[FORWARD_SIGNAL_BASE_FIRST_BYTE + 1] =
      static_cast<uint8_t>((forward_base & GET_SECOND_BYTE) >> 8);
  message.data[BACKWARD_SIGNAL_BASE_FIRST_BYTE] =
      static_cast<uint8_t>(backward_base & GET_FIRST_BYTE);
  message.data[BACKWARD_SIGNAL_BASE_FIRST_BYTE + 1] =
      static_cast<uint8_t>((backward_base & GET_SECOND_BYTE) >> 8);
  // dont care bytes
  message.data[BACKWARD_SIGNAL_BASE_FIRST_BYTE + 2] = 0xFF;
  message.data[BACKWARD_SIGNAL_BASE_FIRST_BYTE + 3] = 0xFF;
  message.data[BACKWARD_SIGNAL_BASE_FIRST_BYTE + 4] = 0xFF;
  _CanHandler.iSendFrame(&message);
}

CartsStatus TTCommControl::GetCartsStatus()
{
  return _Cart;
}

float TTCommControl::fGetFrontCartAngle()
{
  return _fFrontCartAngle;
}

float TTCommControl::fGetBackCartAngle()
{
  return _fBackCartAngle;
}

int TTCommControl::iGetErrorCode()
{
  return _iErrCode;
}

void TTCommControl::vSetPathToAngleDataFile(std::string path_to_file)
{
  path_to_angle_file = path_to_file;
}

bool TTCommControl::bReadAngleDataFile()
{
  std::string line;
  std::string number;
  std::string delimiter_middle = ": ";
  std::string delimiter_endline = ";";
  angle_data_file_input.open(path_to_angle_file + file_name);
  if (!angle_data_file_input.is_open() || angle_data_file_input.fail())
  {
    angle_data_file_input.close();
    printf("\nError : failed to read angle data file content !");
    return false;
  }

  std::getline(angle_data_file_input, line);
  number = line.substr(line.find(delimiter_middle));
  if (number[0] == ':' && number[1] == ' ' && number.back() == ';')
  {
    number = number.substr(2, number.length()-2);
    fFrontCartAngle = std::stof(number);
  }

  std::getline(angle_data_file_input, line);
  number = line.substr(line.find(delimiter_middle));
  if (number[0] == ':' && number[1] == ' ' && number.back() == ';')
  {
    number = number.substr(2, number.length()-2);
    fBackCartAngle = std::stof(number);
    std::cout<<"Back := "<< fBackCartAngle << "\n";
  }
  angle_data_file_input.close();
  return true;
}

bool TTCommControl::bWriteAngleDataFile()
{
  bReadAngleDataFile();
  // save the backup file 
  angle_data_file_output.open((path_to_angle_file + file_name_backup), std::ios::trunc);
  if (!angle_data_file_output.is_open())
  {
    angle_data_file_output.close();
    printf("\nError : failed to read angle data file content !");
    return false;
  }
  angle_data_file_output << "front: " << fFrontCartAngle<<";";
  angle_data_file_output << "back: " << fBackCartAngle<<";";
  // save the proper file
  angle_data_file_output.close();

  angle_data_file_output.open((path_to_angle_file + file_name), std::ios::trunc);
  if (!angle_data_file_output.is_open())
  {
    angle_data_file_output.close();
    printf("\nError : failed to read angle data file content !");
    return false;
  }
  
  return true;
}
