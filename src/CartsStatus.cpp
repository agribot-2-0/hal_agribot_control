/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <hal_ttcontrol_communication/CartsStatus.h>

using std::uint32_t;
using std::uint8_t;

void CartsStatus::setFrontLinearVelocity(const float velocity)
{
  front_linear_velocity_ = velocity;
}

void CartsStatus::setFrontAngularVelocity(const float velocity)
{
  front_angular_velocity_ = velocity;
}

void CartsStatus::setBackLinearVelocity(const float velocity)
{
  back_linear_velocity_ = velocity;
}

void CartsStatus::setBackAngularVelocity(const float velocity)
{
  back_angular_velocity_ = velocity;
}

void CartsStatus::setFrontCartGeometryState(const uint8_t geometry_state)
{
  front_cart_geometry_state_ = geometry_state;
}

void CartsStatus::setBackCartGeometryState(const uint8_t geometry_state)
{
  back_cart_geometry_state_ = geometry_state;
}

float CartsStatus::getFrontLinearVelocity() const
{
  return front_linear_velocity_;
}

float CartsStatus::getFrontAngularVelocity() const
{
  return front_angular_velocity_;
}

float CartsStatus::getBackLinearVelocity() const
{
  return back_linear_velocity_;
}

float CartsStatus::getBackAngularVelocity() const
{
  return back_angular_velocity_;
}

uint8_t CartsStatus::getFrontCartGeometryState() const
{
  return front_cart_geometry_state_;
}

uint8_t CartsStatus::getBackCartGeometryState() const
{
  return back_cart_geometry_state_;
}

void CartsStatus::setTrackEncoderSum(const EncoderTrackId track_id, const int32_t encoder_sum)
{
  switch (track_id)
  {
    case EncoderTrackId::FrontLeft:
      front_left_track_encoder_sum_ = encoder_sum;
      break;
    case EncoderTrackId::FrontRight:
      front_right_track_encoder_sum_ = encoder_sum;
      break;
    case EncoderTrackId::BackLeft:
      back_left_track_encoder_sum_ = encoder_sum;
      break;
    case EncoderTrackId::BackRight:
      back_right_track_encoder_sum_ = encoder_sum;
      break;
    default:
      break;
  }
}

int32_t CartsStatus::getTrackEncoderSum(const EncoderTrackId track_id) const
{
  switch (track_id)
  {
    case EncoderTrackId::FrontLeft:
      return front_left_track_encoder_sum_;
    case EncoderTrackId::FrontRight:
      return front_right_track_encoder_sum_;
    case EncoderTrackId::BackLeft:
      return back_left_track_encoder_sum_;
    case EncoderTrackId::BackRight:
      return back_right_track_encoder_sum_;
    default:
      break;
  }
  return 0;
}

void CartsStatus::setTrackEncoderSpeed(const EncoderTrackId track_id, const int32_t encoder_speed)
{
  switch (track_id)
  {
    case EncoderTrackId::FrontLeft:
      front_left_track_encoder_speed_ = encoder_speed;
      break;
    case EncoderTrackId::FrontRight:
      front_right_track_encoder_speed_ = encoder_speed;
      break;
    case EncoderTrackId::BackLeft:
      back_left_track_encoder_speed_ = encoder_speed;
      break;
    case EncoderTrackId::BackRight:
      back_right_track_encoder_speed_ = encoder_speed;
      break;
    default:
      break;
  }
}

int32_t CartsStatus::getTrackEncoderSpeed(const EncoderTrackId track_id) const
{
  switch (track_id)
  {
    case EncoderTrackId::FrontLeft:
      return front_left_track_encoder_speed_;
    case EncoderTrackId::FrontRight:
      return front_right_track_encoder_speed_;
    case EncoderTrackId::BackLeft:
      return back_left_track_encoder_speed_;
    case EncoderTrackId::BackRight:
      return back_right_track_encoder_speed_;
    default:
      break;
  }
  return 0;
}

void CartsStatus::setTrackEncoderSpeedDesired(const EncoderTrackId track_id, const double encoder_speed)
{
  switch (track_id)
  {
    case EncoderTrackId::FrontLeft:
      front_left_track_encoder_speed_desired_ = encoder_speed;
      break;
    case EncoderTrackId::FrontRight:
      front_right_track_encoder_speed_desired_ = encoder_speed;
      break;
    case EncoderTrackId::BackLeft:
      back_left_track_encoder_speed_desired_ = encoder_speed;
      break;
    case EncoderTrackId::BackRight:
      back_right_track_encoder_speed_desired_ = encoder_speed;
      break;
    default:
      break;
  }
}

double CartsStatus::getTrackEncoderSpeedDesired(const EncoderTrackId track_id) const
{
  switch (track_id)
  {
    case EncoderTrackId::FrontLeft:
      return front_left_track_encoder_speed_desired_;
    case EncoderTrackId::FrontRight:
      return front_right_track_encoder_speed_desired_;
    case EncoderTrackId::BackLeft:
      return back_left_track_encoder_speed_desired_;
    case EncoderTrackId::BackRight:
      return back_right_track_encoder_speed_desired_;
    default:
      break;
  }
  return 0.0;
}

void CartsStatus::setTrackControl(const EncoderTrackId track_id, const int32_t encoder_control)
{
  switch (track_id)
  {
    case EncoderTrackId::FrontLeft:
      front_left_track_control_ = encoder_control;
      break;
    case EncoderTrackId::FrontRight:
      front_right_track_control_ = encoder_control;
      break;
    case EncoderTrackId::BackLeft:
      back_left_track_control_ = encoder_control;
      break;
    case EncoderTrackId::BackRight:
      back_right_track_control_ = encoder_control;
      break;
    default:
      break;
  }
}

int32_t CartsStatus::getTrackControl(const EncoderTrackId track_id) const
{
  switch (track_id)
  {
    case EncoderTrackId::FrontLeft:
      return front_left_track_control_;
    case EncoderTrackId::FrontRight:
      return front_right_track_control_;
    case EncoderTrackId::BackLeft:
      return back_left_track_control_;
    case EncoderTrackId::BackRight:
      return back_right_track_control_;
    default:
      break;
  }
  return 0;
}
