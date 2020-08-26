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

void CartsStatus::vSetFrontLeftEncoderSum(int value)
{
    _iFrontLeftEncoderSum = value;
}

void CartsStatus::vSetFrontRightEncoderSum(int value)
{
    _iFrontRightEncoderSum = value;
}

void CartsStatus::vSetBackLeftEncoderSum(int value)
{
    _iBackLeftEncoderSum = value;
}

void CartsStatus::vSetBackRightEncoderSum(int value)
{
    _iBackRightEncoderSum = value;
}

int CartsStatus::iGetFrontLeftEncoderSum()
{
    return _iFrontLeftEncoderSum;
}

int CartsStatus::iGetFrontRightEncoderSum()
{
    return _iFrontRightEncoderSum;
}

int CartsStatus::iGetBackLeftEncoderSum()
{
    return _iBackLeftEncoderSum;
}

int CartsStatus::iGetBackRightEncoderSum()
{
    return _iBackRightEncoderSum;
}
