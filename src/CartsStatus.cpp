/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <hal_ttcontrol_communication/CartsStatus.h>

void CartsStatus::vSetFrontLinear(float fVelo)
{
  _fFrontLinear = fVelo;
}

void CartsStatus::vSetFrontAngular(float fVelo)
{
  _fFrontAngular = fVelo;
}

void CartsStatus::vSetBackLinear(float fVelo)
{
  _fBackLinear = fVelo;
}

void CartsStatus::vSetBackAngular(float fVelo)
{
  _fBackAngular = fVelo;
}

void CartsStatus::vSetTrackEncoderSum(uint8_t track_ID, int32_t encoder_sum)
{
  switch (track_ID)
  {
    case FRONT_LEFT_TRACK_ENCODER:
      _front_left_track_encoder_sum = encoder_sum;
      break;
    case FRONT_RIGHT_TRACK_ENCODER:
      _front_right_track_encoder_sum = encoder_sum;
      break;
    case BACK_LEFT_TRACK_ENCODER:
      _back_left_track_encoder_sum = encoder_sum;
      break;
    case BACK_RIGHT_TRACK_ENCODER:
      _back_right_track_encoder_sum = encoder_sum;
      break;
    default:
      break;
  }
}

int32_t CartsStatus::iGetTrackEncoderSum(uint8_t track_ID)
{
  switch (track_ID)
  {
    case FRONT_LEFT_TRACK_ENCODER:
      return _front_left_track_encoder_sum;
    case FRONT_RIGHT_TRACK_ENCODER:
      return _front_right_track_encoder_sum;
    case BACK_LEFT_TRACK_ENCODER:
      return _back_left_track_encoder_sum;
    case BACK_RIGHT_TRACK_ENCODER:
      return _back_right_track_encoder_sum;
    default:
      break;
  }
  return 0;
}

float CartsStatus::fGetFrontLinear()
{
  return _fFrontLinear;
}

float CartsStatus::fGetFrontAngular()
{
  return _fFrontAngular;
}

float CartsStatus::fGetBackLinear()
{
  return _fBackLinear;
}

float CartsStatus::fGetBackAngular()
{
  return _fBackAngular;
}
