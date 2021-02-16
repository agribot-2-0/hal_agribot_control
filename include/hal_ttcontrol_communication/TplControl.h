/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <cstdint>

struct TplControl
{
  std::int16_t pto_valve_control;
  std::int16_t red_qcf_valve_control;
  std::int16_t green_qcf_valve_control;
  std::int16_t blue_qcf_valve_control;
  std::int16_t height_actuator_control;
  std::int16_t angle_actuator_control;
};
