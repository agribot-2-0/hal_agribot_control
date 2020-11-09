/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_msgs/AgribotCaterpillar.h>
#include <hal_ttcontrol_communication/TTCommControl.h>
#include <dynamic_reconfigure/server.h>
#include <hal_agribot_control/TTControlParametersConfig.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

constexpr char used_can[] = "can0";
constexpr uint WAIT_FOR_TTCONTROL = 100000;

constexpr uint8_t TTCONTROL_SIGNAL_PARAMS_FLAG = 0x01;
constexpr uint8_t TTCONTROL_FRONT_PID_T_PARAMS_FLAG = 0x02;
constexpr uint8_t TTCONTROL_FRONT_PID_R_PARAMS_FLAG = 0x04;
constexpr uint8_t TTCONTROL_BACK_PID_T_PARAMS_FLAG = 0x08;
constexpr uint8_t TTCONTROL_BACK_PID_R_PARAMS_FLAG = 0x10;

TTCommControl CommHandlerControl(used_can);

void callbackDynamicReconfigure(const hal_agribot_control::TTControlParametersConfig& config, uint32_t level)
{
  if (level & TTCONTROL_SIGNAL_PARAMS_FLAG)
  {
    CommHandlerControl.vSendSignalsParameters(config.ditter_period_int, config.ditter_amplitude_int,
                                              config.encoder_period_int);
  }
  if (level & TTCONTROL_FRONT_PID_T_PARAMS_FLAG)
  {
    CommHandlerControl.vSendCartsPIDParameters(config.front_kp_t_double, config.front_kd_t_double,
                                               config.front_acc_t_double,
                                               eTTControlParamsType::TTCONTROL_FRONT_PID_T_PARAMS_TYPE);
  }
  if (level & TTCONTROL_FRONT_PID_R_PARAMS_FLAG)
  {
    CommHandlerControl.vSendCartsPIDParameters(config.front_kp_r_double, config.front_kd_r_double,
                                               config.front_acc_r_double,
                                               eTTControlParamsType::TTCONTROL_FRONT_PID_R_PARAMS_TYPE);
  }
  if (level & TTCONTROL_BACK_PID_T_PARAMS_FLAG)
  {
    CommHandlerControl.vSendCartsPIDParameters(config.back_kp_t_double, config.back_kd_t_double,
                                               config.back_acc_t_double,
                                               eTTControlParamsType::TTCONTROL_BACK_PID_T_PARAMS_TYPE);
  }
  if (level & TTCONTROL_BACK_PID_R_PARAMS_FLAG)
  {
    CommHandlerControl.vSendCartsPIDParameters(config.back_kp_r_double, config.back_kd_r_double,
                                               config.back_acc_r_double,
                                               eTTControlParamsType::TTCONTROL_BACK_PID_R_PARAMS_TYPE);
  }
}

void frontCallback(const agribot_msgs::AgribotCaterpillar& msg)
{
  CommHandlerControl.vSendVelocities(msg.front_linear, msg.front_angular, msg.back_linear, msg.back_angular);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "agribot_control");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Subscriber sub = n.subscribe("/agribot/control/caterpillar", 1000, frontCallback);
  ros::Publisher CartsStatusPublisher =
      n.advertise<agribot_msgs::AgribotCaterpillar>("/agribot/encoder/caterpillar", 1);
  ros::Publisher FrontCartAnglePublisher = n.advertise<std_msgs::Float32>("/agribot/encoder/cart/front", 1);
  ros::Publisher BackCartAnglePublisher = n.advertise<std_msgs::Float32>("/agribot/encoder/cart/back", 1);
  agribot_msgs::AgribotCaterpillar AgribotCaterpillarMsg;
  dynamic_reconfigure::Server<hal_agribot_control::TTControlParametersConfig> server;
  dynamic_reconfigure::Server<hal_agribot_control::TTControlParametersConfig>::CallbackType callback_function_object;
  std_msgs::Float32 FrontAngleMsg;
  std_msgs::Float32 BackAngleMsg;
  callback_function_object = boost::bind(&callbackDynamicReconfigure, _1, _2);
  server.setCallback(callback_function_object);
  uint8_t u8Status = 0x00;
  while (ros::ok())
  {
    CommHandlerControl.vReceiveCanFrames();
    u8Status = CommHandlerControl.u8ParseCanFrames();
    // if velociteis frames received in u8ParseCanFrames() method
    if ((u8Status & VELOCITIES_MASK) == VELOCITIES_MASK)
    {
      CartsStatus cartStatus = CommHandlerControl.GetCartsStatus();
      AgribotCaterpillarMsg.header.stamp = ros::Time::now();
      AgribotCaterpillarMsg.header.frame_id = "hal_caterpillar_feedback";
      AgribotCaterpillarMsg.front_linear = cartStatus.fGetFrontLinear();
      AgribotCaterpillarMsg.front_angular = cartStatus.fGetFrontAngular();
      AgribotCaterpillarMsg.back_linear = cartStatus.fGetBackLinear();
      AgribotCaterpillarMsg.back_angular = cartStatus.fGetBackAngular();
      CartsStatusPublisher.publish(AgribotCaterpillarMsg);
    }
    // if front cart angle frames received in u8ParseCanFrames() method
    if ((u8Status & FRONT_ANGLE_MASK) == FRONT_ANGLE_MASK)
    {
      FrontAngleMsg.data = CommHandlerControl.fGetFrontCartAngle();
      FrontCartAnglePublisher.publish(FrontAngleMsg);
    }
    // if back cart angle frames received in u8ParseCanFrames() method
    if ((u8Status & BACK_ANGLE_MASK) == BACK_ANGLE_MASK)
    {
      BackAngleMsg.data = CommHandlerControl.fGetBackCartAngle();
      BackCartAnglePublisher.publish(BackAngleMsg);
    }

    // if automatic state request frame received in u8ParseCanFrames()
    // method
    if ((u8Status & AUTOMATIC_STATE_REQUEST_MASK) == AUTOMATIC_STATE_REQUEST_MASK)
    {
      // TODO(Michal/Adam): Implement some diagnostics
      ROS_INFO("Received automatic mode request");
      usleep(WAIT_FOR_TTCONTROL);
      CommHandlerControl.vConfirmAdvantechReadiness();
    }
    // if automatic state request error frame received in u8ParseCanFrames()
    // method
    if ((u8Status & AUTOMATIC_STATE_REQUEST_ERROR_MASK) == AUTOMATIC_STATE_REQUEST_ERROR_MASK)
    {
      ROS_ERROR("Automatic state request error: %d", CommHandlerControl.iGetErrorCode());
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
