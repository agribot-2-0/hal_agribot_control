/*
* Copyright (C) 2020 Agribot. All rights reserved.
*/

#pragma once
#include <stdint.h>

constexpr uint8_t DELTAS_GEOMETRY_STATE_UNKNOWN = 0x02;
constexpr uint8_t DELTAS_GEOMETRY_STATE_CHANGED = 0x01;
constexpr uint8_t DELTAS_GEOMETRY_STATE_NORMAL = 0x00;

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
    void vSetFrontLinear(float fVelo);
    void vSetFrontAngular(float fVelo);
    void vSetBackLinear(float fVelo);
    void vSetBackAngular(float fVelo);
    void vSetFrontCartGeometryState(uint8_t geometry_state);
    void vSetBackCartGeometryState(uint8_t geometry_state);

    float fGetFrontLinear();
    float fGetFrontAngular();
    float fGetBackLinear();
    float fGetBackAngular();
    uint8_t bGetFrontCartGeometryState() const;
    uint8_t bGetBackCartGeometryState() const;
};
