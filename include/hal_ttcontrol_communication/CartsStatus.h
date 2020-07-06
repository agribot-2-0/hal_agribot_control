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
 public:
    void vSetFrontLinear(float fVelo);
    void vSetFrontAngular(float fVelo);
    void vSetBackLinear(float fVelo);
    void vSetBackAngular(float fVelo);

    float fGetFrontLinear();
    float fGetFrontAngular();
    float fGetBackLinear();
    float fGetBackAngular();
};
