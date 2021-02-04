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

void CartsStatus::setTrackEncoderSum(const uint8_t track_ID, const int32_t encoder_sum)
{
  switch (track_ID)
  {
    case eEncoderCaterpillarID::FRONT_LEFT_TRACK_ENCODER:
      front_left_track_encoder_sum_ = encoder_sum;
      break;
    case eEncoderCaterpillarID::FRONT_RIGHT_TRACK_ENCODER:
      front_right_track_encoder_sum_ = encoder_sum;
      break;
    case eEncoderCaterpillarID::BACK_LEFT_TRACK_ENCODER:
      back_left_track_encoder_sum_ = encoder_sum;
      break;
    case eEncoderCaterpillarID::BACK_RIGHT_TRACK_ENCODER:
      back_right_track_encoder_sum_ = encoder_sum;
      break;
    default:
      break;
  }
}

int32_t CartsStatus::getTrackEncoderSum(const uint8_t track_ID) const
{
  switch (track_ID)
  {
    case eEncoderCaterpillarID::FRONT_LEFT_TRACK_ENCODER:
      return front_left_track_encoder_sum_;
    case eEncoderCaterpillarID::FRONT_RIGHT_TRACK_ENCODER:
      return front_right_track_encoder_sum_;
    case eEncoderCaterpillarID::BACK_LEFT_TRACK_ENCODER:
      return back_left_track_encoder_sum_;
    case eEncoderCaterpillarID::BACK_RIGHT_TRACK_ENCODER:
      return back_right_track_encoder_sum_;
    default:
      break;
  }
  return 0;
}

float CartsStatus::fGetFrontLinear() const
{
  return _fFrontLinear;
}

float CartsStatus::fGetFrontAngular() const
{
  return _fFrontAngular;
}

float CartsStatus::fGetBackLinear() const
{
  return _fBackLinear;
}

float CartsStatus::fGetBackAngular() const
{
  return _fBackAngular;
}
