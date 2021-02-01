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

void CartsStatus::vSetFrontCartGeometryState(uint8_t geometry_state)
{
    front_cart_geometry_state_ = geometry_state;
}

void CartsStatus::vSetBackCartGeometryState(uint8_t geometry_state)
{
    back_cart_geometry_state_ = geometry_state;
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

uint8_t CartsStatus::bGetFrontCartGeometryState() const
{
    return front_cart_geometry_state_;
}

uint8_t CartsStatus::bGetBackCartGeometryState() const
{
    return back_cart_geometry_state_;
}
