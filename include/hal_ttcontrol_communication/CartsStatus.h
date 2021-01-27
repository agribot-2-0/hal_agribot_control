/*
* Copyright (C) 2020 Agribot. All rights reserved.
*/

#pragma once
#include <cstdint>

constexpr uint8_t FRONT_LEFT_TRACK_ENCODER = 0x01;
constexpr uint8_t FRONT_RIGHT_TRACK_ENCODER = 0x02;
constexpr uint8_t BACK_LEFT_TRACK_ENCODER = 0x04;
constexpr uint8_t BACK_RIGHT_TRACK_ENCODER = 0x08;

class CartsStatus
{
 private:
    float _fFrontLinear;
    float _fFrontAngular;
    float _fBackLinear;
    float _fBackAngular;
    int32_t _front_left_track_encoder_sum;
    int32_t _front_right_track_encoder_sum;
    int32_t _back_left_track_encoder_sum;
    int32_t _back_right_track_encoder_sum;
 public:
    void vSetFrontLinear(float fVelo);
    void vSetFrontAngular(float fVelo);
    void vSetBackLinear(float fVelo);
    void vSetBackAngular(float fVelo);
    void vSetTrackEncoderSum(uint8_t track_ID, int32_t encoder_sum);

    float fGetFrontLinear();
    float fGetFrontAngular();
    float fGetBackLinear();
    float fGetBackAngular();
    int32_t iGetTrackEncoderSum(uint8_t track_ID);
};
