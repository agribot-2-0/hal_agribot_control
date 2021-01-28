/*
* Copyright (C) 2020 Agribot. All rights reserved.
*/

#pragma once

class CartsStatus
{
 private:
    float _fFrontLinear;
    float _fFrontAngular;
    float _fBackLinear;
    float _fBackAngular;
    bool front_cart_geometry_state;
    bool back_cart_geometry_state;
 public:
    void vSetFrontLinear(float fVelo);
    void vSetFrontAngular(float fVelo);
    void vSetBackLinear(float fVelo);
    void vSetBackAngular(float fVelo);
    void vSetFrontCartGeometryState(bool geometry_state);
    void vSetBackCartGeometryState(bool geometry_state);

    float fGetFrontLinear();
    float fGetFrontAngular();
    float fGetBackLinear();
    float fGetBackAngular();
    bool bGetFrontCartGeometryState();
    bool bGetBackCartGeometryState();
};
