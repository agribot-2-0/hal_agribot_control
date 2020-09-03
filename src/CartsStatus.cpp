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

double CartsStatus::iGetFrontLeftEncoderSum()
{
  return impulsesToRadians(_iFrontLeftEncoderSum);
}

double CartsStatus::iGetFrontRightEncoderSum()
{
  return impulsesToRadians(_iFrontRightEncoderSum);
}

double CartsStatus::iGetBackLeftEncoderSum()
{
  return impulsesToRadians(_iBackLeftEncoderSum);
}

double CartsStatus::iGetBackRightEncoderSum()
{
  return impulsesToRadians(_iBackRightEncoderSum);
}

void CartsStatus::setImpulsesPerRevolution(int impulses)
{
  if (impulses < 0)
  {
    _impulsesPerRevolution = 1;
  }
  else
  {
    _impulsesPerRevolution = impulses;
  }
}

double CartsStatus::impulsesToRadians(int impulses)
{
  return (2 * PI * impulses) / _impulsesPerRevolution;
}
