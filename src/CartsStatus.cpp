/*
* Copyright (C) 2020 Agribot. All rights reserved.
*/

#include <hal_ttcontrol_communication/CartsStatus.h>

void CartsStatus::setFrontLinear(const float velo)
{
    front_linear_ = velo;
}

void CartsStatus::setFrontAngular(const float velo)
{
    front_angular_ = velo;
}

void CartsStatus::setBackLinear(const float velo)
{
    back_linear_ = velo;
}

void CartsStatus::setBackAngular(const float velo)
{
    back_angular_ = velo;
}

void CartsStatus::setFrontCartGeometryState(const uint8_t geometry_state)
{
    front_cart_geometry_state_ = geometry_state;
}

void CartsStatus::setBackCartGeometryState(const uint8_t geometry_state)
{
    back_cart_geometry_state_ = geometry_state;
}

float CartsStatus::getFrontLinear() const
{
    return front_linear_;
}

float CartsStatus::getFrontAngular() const
{
    return front_angular_;
}

float CartsStatus::getBackLinear() const
{
    return back_linear_;
}

float CartsStatus::getBackAngular() const
{
    return back_angular_;
}

uint8_t CartsStatus::getFrontCartGeometryState() const
{
    return front_cart_geometry_state_;
}

uint8_t CartsStatus::getBackCartGeometryState() const
{
    return back_cart_geometry_state_;
}
