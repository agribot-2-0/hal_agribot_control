/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <hal_ttcontrol_communication/CartsStatus.h>

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
