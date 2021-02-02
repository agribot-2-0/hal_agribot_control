/*
* Copyright (C) 2020 Agribot. All rights reserved.
*/

#pragma once
#include <cstdint>

enum eEncoderCaterpillarID : uint8_t
{
   FRONT_LEFT_TRACK_ENCODER = 0x01,
   FRONT_RIGHT_TRACK_ENCODER = 0x02,
   BACK_LEFT_TRACK_ENCODER = 0x04,
   BACK_RIGHT_TRACK_ENCODER = 0x08
};

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
    void vSetTrackEncoderSum(const uint8_t track_ID, const int32_t encoder_sum);

    float fGetFrontLinear() const;
    float fGetFrontAngular() const;
    float fGetBackLinear() const;
    float fGetBackAngular() const;
    int32_t iGetTrackEncoderSum(const uint8_t track_ID) const;
};
