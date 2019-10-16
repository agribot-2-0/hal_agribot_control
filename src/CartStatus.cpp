#include <CartStatus.h>

void CartStatus::vSetLeftTrackVelo(float fVelo)
{
    _fLeftTrackVelocity = fVelo;
}

void CartStatus::vSetRightTrackVelo(float fVelo)
{
    _fRightTrackVelocity = fVelo;
}
void CartStatus::vSetAngle(float fAngl)
{
    _fAngle=fAngl;
}
float CartStatus::fGetLeftTrackVelo()
{
    return _fLeftTrackVelocity;
}
float CartStatus::fGetRightTrackVelo()
{
    return _fRightTrackVelocity;
}
float CartStatus::fGetAngle()
{
    return _fAngle;
}