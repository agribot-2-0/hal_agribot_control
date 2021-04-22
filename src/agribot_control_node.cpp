/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */
#include <hal_ttcontrol_communication/AgribotControlRosWrapper.h>
#include <ros/console.h>
#include <cstdint>

int main(int argc, char** argv)
{
  try
  {
    AgribotControlRosWrapper agribot_control_wrapper(argc, argv, "agribot_control");
    agribot_control_wrapper.execute();
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM("[HAL_AGRIBOT_CONTROL] " << ex.what());
  }
}
