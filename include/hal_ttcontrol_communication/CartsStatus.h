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
    int _iFrontLeftEncoderSum;
    int _iFrontRightEncoderSum;
    int _iBackLeftEncoderSum;
    int _iBackRightEncoderSum;
 public:
    void vSetFrontLinear(float fVelo);
    void vSetFrontAngular(float fVelo);
    void vSetBackLinear(float fVelo);
    void vSetBackAngular(float fVelo);
    void vSetFrontLeftEncoderSum(int value);
    void vSetFrontRightEncoderSum(int value);
    void vSetBackLeftEncoderSum(int value);
    void vSetBackRightEncoderSum(int value);

    float fGetFrontLinear();
    float fGetFrontAngular();
    float fGetBackLinear();
    float fGetBackAngular();
    int iGetFrontLeftEncoderSum();
    int iGetFrontRightEncoderSum();
    int iGetBackLeftEncoderSum();
    int iGetBackRightEncoderSum();
};
