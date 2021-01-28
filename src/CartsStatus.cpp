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

void CartsStatus::vSetFrontCartGeometryState(bool geometry_state)
{
    front_cart_geometry_state = geometry_state;
}

void CartsStatus::vSetBackCartGeometryState(bool geometry_state)
{
    back_cart_geometry_state = geometry_state;
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

bool CartsStatus::bGetFrontCartGeometryState()
{
    return front_cart_geometry_state;
}

bool CartsStatus::bGetBackCartGeometryState()
{
    return back_cart_geometry_state;
}
