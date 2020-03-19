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
