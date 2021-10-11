/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once
#include <cstdint>

enum class EncoderTrackId
{
  FrontLeft = 0x01,
  FrontRight = 0x02,
  BackLeft = 0x04,
  BackRight = 0x08
};

class CartsStatus
{
private:
  float front_linear_velocity_;
  float front_angular_velocity_;
  float back_linear_velocity_;
  float back_angular_velocity_;
  std::uint8_t front_cart_geometry_state_;
  std::uint8_t back_cart_geometry_state_;
  std::int32_t front_left_track_encoder_time_diff_ = 1.0;
  std::int32_t front_right_track_encoder_time_diff_ = 1.0;
  std::int32_t back_left_track_encoder_time_diff_ = 1.0;
  std::int32_t back_right_track_encoder_time_diff_ = 1.0;
  std::int32_t front_left_track_encoder_speed_ = 0.0;
  std::int32_t front_right_track_encoder_speed_ = 0.0;
  std::int32_t back_left_track_encoder_speed_ = 0.0;
  std::int32_t back_right_track_encoder_speed_ = 0.0;
  double front_left_track_encoder_speed_desired_ = 0.0;
  double front_right_track_encoder_speed_desired_ = 0.0;
  double back_left_track_encoder_speed_desired_ = 0.0;
  double back_right_track_encoder_speed_desired_ = 0.0;
  std::int32_t front_left_track_control_ = 0.0;
  std::int32_t front_right_track_control_ = 0.0;
  std::int32_t back_left_track_control_ = 0.0;
  std::int32_t back_right_track_control_ = 0.0;

public:
  void setFrontLinearVelocity(const float velocity);
  void setFrontAngularVelocity(const float velocity);
  void setBackLinearVelocity(const float velocity);
  void setBackAngularVelocity(const float velocity);
  void setFrontCartGeometryState(const std::uint8_t geometry_state);
  void setBackCartGeometryState(const std::uint8_t geometry_state);
  void setTrackEncoderTimeDiff(const EncoderTrackId track_id, const std::int32_t encoder_sum);
  void setTrackEncoderSpeed(const EncoderTrackId track_id, const std::int32_t encoder_speed);
  void setTrackEncoderSpeedDesired(const EncoderTrackId track_id, const double encoder_speed);
  void setTrackControl(const EncoderTrackId track_id, const std::int32_t encoder_control);

  float getFrontLinearVelocity() const;
  float getFrontAngularVelocity() const;
  float getBackLinearVelocity() const;
  float getBackAngularVelocity() const;
  std::uint8_t getFrontCartGeometryState() const;
  std::uint8_t getBackCartGeometryState() const;
  std::int32_t getTrackEncoderTimeDiff(const EncoderTrackId track_id) const;
  std::int32_t getTrackEncoderSpeed(const EncoderTrackId track_id) const;
  double getTrackEncoderSpeedDesired(const EncoderTrackId track_id) const;
  std::int32_t getTrackControl(const EncoderTrackId track_id) const;
};
