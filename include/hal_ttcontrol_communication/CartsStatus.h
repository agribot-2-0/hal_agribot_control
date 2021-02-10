/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once
#include <cstdint>

enum class EncoderTrackId
{
  front_left = 0x01,
  front_right = 0x02,
  back_left = 0x04,
  back_right = 0x08
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
  std::int32_t front_left_track_encoder_sum_;
  std::int32_t front_right_track_encoder_sum_;
  std::int32_t back_left_track_encoder_sum_;
  std::int32_t back_right_track_encoder_sum_;

public:
  void setFrontLinearVelocity(const float velocity);
  void setFrontAngularVelocity(const float velocity);
  void setBackLinearVelocity(const float velocity);
  void setBackAngularVelocity(const float velocity);
  void setFrontCartGeometryState(const std::uint8_t geometry_state);
  void setBackCartGeometryState(const std::uint8_t geometry_state);
  void setTrackEncoderSum(const EncoderTrackId track_id, const std::int32_t encoder_sum);

  float getFrontLinearVelocity() const;
  float getFrontAngularVelocity() const;
  float getBackLinearVelocity() const;
  float getBackAngularVelocity() const;
  std::uint8_t getFrontCartGeometryState() const;
  std::uint8_t getBackCartGeometryState() const;
  std::int32_t getTrackEncoderSum(const EncoderTrackId track_id) const;
};
