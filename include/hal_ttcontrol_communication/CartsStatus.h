/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once
#include <stdint.h>

class CartsStatus
{
private:
  float front_linear_;
  float front_angular_;
  float back_linear_;
  float back_angular_;
  uint8_t front_cart_geometry_state_;
  uint8_t back_cart_geometry_state_;

public:
  void setFrontLinear(const float velo);
  void setFrontAngular(const float velo);
  void setBackLinear(const float velo);
  void setBackAngular(const float velo);
  void setFrontCartGeometryState(const uint8_t geometry_state);
  void setBackCartGeometryState(const uint8_t geometry_state);

  float getFrontLinear() const;
  float getFrontAngular() const;
  float getBackLinear() const;
  float getBackAngular() const;
  uint8_t getFrontCartGeometryState() const;
  uint8_t getBackCartGeometryState() const;
};
