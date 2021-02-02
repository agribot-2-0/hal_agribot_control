/*
* Copyright (C) 2020 Agribot. All rights reserved.
*/

#pragma once
#include <stdint.h>

enum eDeltasGeometryState : uint8_t
{
   DELTAS_GEOMETRY_STATE_UNKNOWN = 0x02,
   DELTAS_GEOMETRY_STATE_CHANGED = 0x01,
   DELTAS_GEOMETRY_STATE_NORMAL = 0x00
};

class CartsStatus
{
 private:
    float _fFrontLinear;
    float _fFrontAngular;
    float _fBackLinear;
    float _fBackAngular;
    uint8_t front_cart_geometry_state_;
    uint8_t back_cart_geometry_state_;
 public:
    void vSetFrontLinear(const float fVelo);
    void vSetFrontAngular(const float fVelo);
    void vSetBackLinear(const float fVelo);
    void vSetBackAngular(const float fVelo);
    void vSetFrontCartGeometryState(const uint8_t geometry_state);
    void vSetBackCartGeometryState(const uint8_t geometry_state);

    float fGetFrontLinear()const;
    float fGetFrontAngular() const;
    float fGetBackLinear() const;
    float fGetBackAngular() const;
    uint8_t bGetFrontCartGeometryState() const;
    uint8_t bGetBackCartGeometryState() const;
};
