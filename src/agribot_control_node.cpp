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
#include <std_msgs/String.h>
#include <string_view>

constexpr char used_can[] = "can0";
constexpr uint WAIT_FOR_TTCONTROL = 100000;
constexpr bool READ_BACKUP_CARTS_ENCODERS_FILE = false;
constexpr bool READ_REGULAR_CARTS_ENCODERS_FILE = true;
constexpr std::string_view TTCONTROL_CONNECTION_MSG = "connected";
constexpr bool TTCONTROL_CONNECTED = true;
constexpr bool TTCONTROL_DISCONNECTED = false;

const struct TTCONTROL_PARAMETER_FLAGS
{
  uint32_t TTCONTROL_SIGNAL_PARAMS_FLAG = 0x01;
  uint32_t TTCONTROL_FRONT_PID_T_PARAMS_FLAG = 0x02;
  uint32_t TTCONTROL_FRONT_PID_R_PARAMS_FLAG = 0x04;
  uint32_t TTCONTROL_BACK_PID_T_PARAMS_FLAG = 0x08;
  uint32_t TTCONTROL_BACK_PID_R_PARAMS_FLAG = 0x10;
  uint32_t TTCONTROL_FRONT_LEFT_CATERPILLAR_PID_PARAMS_FLAG = 0x20;
  uint32_t TTCONTROL_FRONT_LEFT_CATERPILLAR_SIGNALS_BASE_FLAG = 0x40;
  uint32_t TTCONTROL_FRONT_RIGHT_CATERPILLAR_PID_PARAMS_FLAG = 0x80;
  uint32_t TTCONTROL_FRONT_RIGHT_CATERPILLAR_SIGNALS_BASE_FLAG = 0x100;
  uint32_t TTCONTROL_BACK_LEFT_CATERPILLAR_PID_PARAMS_FLAG = 0x200;
  uint32_t TTCONTROL_BACK_LEFT_CATERPILLAR_SIGNALS_BASE_FLAG = 0x400;
  uint32_t TTCONTROL_BACK_RIGHT_CATERPILLAR_PID_PARAMS_FLAG = 0x800;
  uint32_t TTCONTROL_BACK_RIGHT_CATERPILLAR_SIGNALS_BASE_FLAG = 0x1000;
} TTControl_param_flags;

TTCommControl CommHandlerControl(used_can);

void callbackDynamicReconfigure(const hal_agribot_control::TTControlParametersConfig& config, uint32_t level)
{
  // maximum number of queed msg to read in TTControl for a given CAN_ID is 10, so we need to send the msgs
  // in turns
  if (level == 0xFFFFFFFF)
  {
    CommHandlerControl.vSendSignalsParameters(config.ditter_period_int, config.ditter_amplitude_int,
                                              config.encoder_period_int);
    CommHandlerControl.vSendCartsPIDParameters(
        config.front_kp_t_double, config.front_ki_t_double, config.front_kd_t_double, config.front_acc_t_double,
        CommHandlerControl.TTControlParamsFrameType.TTCONTROL_FRONT_PID_T_PARAMS_TYPE);
    CommHandlerControl.vSendCartsPIDParameters(
        config.front_kp_r_double, config.front_ki_r_double, config.front_kd_r_double, config.front_acc_r_double,
        CommHandlerControl.TTControlParamsFrameType.TTCONTROL_FRONT_PID_R_PARAMS_TYPE);
    CommHandlerControl.vSendCartsPIDParameters(
        config.back_kp_t_double, config.back_ki_t_double, config.back_kd_t_double, config.back_acc_t_double,
        CommHandlerControl.TTControlParamsFrameType.TTCONTROL_BACK_PID_T_PARAMS_TYPE);
    CommHandlerControl.vSendCartsPIDParameters(
        config.back_kp_r_double, config.back_ki_r_double, config.back_kd_r_double, config.back_acc_r_double,
        CommHandlerControl.TTControlParamsFrameType.TTCONTROL_BACK_PID_R_PARAMS_TYPE);
    CommHandlerControl.vSendCaterpillarPIDParameters(
        config.front_left_caterpillar_kp, config.front_left_caterpillar_ki, config.front_left_caterpillar_kd,
        CommHandlerControl.TTControlParamsFrameType.TTCONTROL_FRONT_LEFT_CATERPILLAR_PID_PARAMS_TYPE);

    ros::Duration(0.5).sleep();  // sleep for half a second, to give TTControl time to read and save the parameters
    CommHandlerControl.vSendCaterpillarBaseSignal(
        config.front_left_caterpillar_signal_base_forward, config.front_left_caterpillar_signal_base_backward,
        CommHandlerControl.TTControlParamsFrameType.TTCONTROL_FRONT_LEFT_CATERPILLAR_SIGNAL_BASE_TYPE);
    CommHandlerControl.vSendCaterpillarPIDParameters(
        config.front_right_caterpillar_kp, config.front_right_caterpillar_ki, config.front_right_caterpillar_kd,
        CommHandlerControl.TTControlParamsFrameType.TTCONTROL_FRONT_RIGHT_CATERPILLAR_PID_PARAMS_TYPE);
    CommHandlerControl.vSendCaterpillarBaseSignal(
        config.front_right_caterpillar_signal_base_forward, config.front_right_caterpillar_signal_base_backward,
        CommHandlerControl.TTControlParamsFrameType.TTCONTROL_FRONT_RIGHT_CATERPILLAR_SIGNAL_BASE_TYPE);
    CommHandlerControl.vSendCaterpillarPIDParameters(
        config.back_left_caterpillar_kp, config.back_left_caterpillar_ki, config.back_left_caterpillar_kd,
        CommHandlerControl.TTControlParamsFrameType.TTCONTROL_BACK_LEFT_CATERPILLAR_PID_PARAMS_TYPE);
    CommHandlerControl.vSendCaterpillarBaseSignal(
        config.back_left_caterpillar_signal_base_forward, config.back_left_caterpillar_signal_base_backward,
        CommHandlerControl.TTControlParamsFrameType.TTCONTROL_BACK_LEFT_CATERPILLAR_SIGNAL_BASE_TYPE);
    CommHandlerControl.vSendCaterpillarPIDParameters(
        config.back_right_caterpillar_kp, config.back_right_caterpillar_ki, config.back_right_caterpillar_kd,
        CommHandlerControl.TTControlParamsFrameType.TTCONTROL_BACK_RIGHT_CATERPILLAR_PID_PARAMS_TYPE);
    CommHandlerControl.vSendCaterpillarBaseSignal(
        config.back_right_caterpillar_signal_base_forward, config.back_right_caterpillar_signal_base_backward,
        CommHandlerControl.TTControlParamsFrameType.TTCONTROL_BACK_RIGHT_CATERPILLAR_SIGNAL_BASE_TYPE);
  }
  else
  {
    if (level & TTControl_param_flags.TTCONTROL_SIGNAL_PARAMS_FLAG)
    {
      CommHandlerControl.vSendSignalsParameters(config.ditter_period_int, config.ditter_amplitude_int,
                                                config.encoder_period_int);
    }
    if (level & TTControl_param_flags.TTCONTROL_FRONT_PID_T_PARAMS_FLAG)
    {
      CommHandlerControl.vSendCartsPIDParameters(
          config.front_kp_t_double, config.front_ki_t_double, config.front_kd_t_double, config.front_acc_t_double,
          CommHandlerControl.TTControlParamsFrameType.TTCONTROL_FRONT_PID_T_PARAMS_TYPE);
    }
    if (level & TTControl_param_flags.TTCONTROL_FRONT_PID_R_PARAMS_FLAG)
    {
      CommHandlerControl.vSendCartsPIDParameters(
          config.front_kp_r_double, config.front_ki_r_double, config.front_kd_r_double, config.front_acc_r_double,
          CommHandlerControl.TTControlParamsFrameType.TTCONTROL_FRONT_PID_R_PARAMS_TYPE);
    }
    if (level & TTControl_param_flags.TTCONTROL_BACK_PID_T_PARAMS_FLAG)
    {
      CommHandlerControl.vSendCartsPIDParameters(
          config.back_kp_t_double, config.back_ki_t_double, config.back_kd_t_double, config.back_acc_t_double,
          CommHandlerControl.TTControlParamsFrameType.TTCONTROL_BACK_PID_T_PARAMS_TYPE);
    }
    if (level & TTControl_param_flags.TTCONTROL_BACK_PID_R_PARAMS_FLAG)
    {
      CommHandlerControl.vSendCartsPIDParameters(
          config.back_kp_r_double, config.back_ki_r_double, config.back_kd_r_double, config.back_acc_r_double,
          CommHandlerControl.TTControlParamsFrameType.TTCONTROL_BACK_PID_R_PARAMS_TYPE);
    }
    if (level & TTControl_param_flags.TTCONTROL_FRONT_LEFT_CATERPILLAR_PID_PARAMS_FLAG)
    {
      CommHandlerControl.vSendCaterpillarPIDParameters(
          config.front_left_caterpillar_kp, config.front_left_caterpillar_ki, config.front_left_caterpillar_kd,
          CommHandlerControl.TTControlParamsFrameType.TTCONTROL_FRONT_LEFT_CATERPILLAR_PID_PARAMS_TYPE);
    }
    if (level & TTControl_param_flags.TTCONTROL_FRONT_LEFT_CATERPILLAR_SIGNALS_BASE_FLAG)
    {
      CommHandlerControl.vSendCaterpillarBaseSignal(
          config.front_left_caterpillar_signal_base_forward, config.front_left_caterpillar_signal_base_backward,
          CommHandlerControl.TTControlParamsFrameType.TTCONTROL_FRONT_LEFT_CATERPILLAR_SIGNAL_BASE_TYPE);
    }
    if (level & TTControl_param_flags.TTCONTROL_FRONT_RIGHT_CATERPILLAR_PID_PARAMS_FLAG)
    {
      CommHandlerControl.vSendCaterpillarPIDParameters(
          config.front_right_caterpillar_kp, config.front_right_caterpillar_ki, config.front_right_caterpillar_kd,
          CommHandlerControl.TTControlParamsFrameType.TTCONTROL_FRONT_RIGHT_CATERPILLAR_PID_PARAMS_TYPE);
    }
    if (level & TTControl_param_flags.TTCONTROL_FRONT_RIGHT_CATERPILLAR_SIGNALS_BASE_FLAG)
    {
      CommHandlerControl.vSendCaterpillarBaseSignal(
          config.front_right_caterpillar_signal_base_forward, config.front_right_caterpillar_signal_base_backward,
          CommHandlerControl.TTControlParamsFrameType.TTCONTROL_FRONT_RIGHT_CATERPILLAR_SIGNAL_BASE_TYPE);
    }
    if (level & TTControl_param_flags.TTCONTROL_BACK_LEFT_CATERPILLAR_PID_PARAMS_FLAG)
    {
      CommHandlerControl.vSendCaterpillarPIDParameters(
          config.back_left_caterpillar_kp, config.back_left_caterpillar_ki, config.back_left_caterpillar_kd,
          CommHandlerControl.TTControlParamsFrameType.TTCONTROL_BACK_LEFT_CATERPILLAR_PID_PARAMS_TYPE);
    }
    if (level & TTControl_param_flags.TTCONTROL_BACK_LEFT_CATERPILLAR_SIGNALS_BASE_FLAG)
    {
      CommHandlerControl.vSendCaterpillarBaseSignal(
          config.back_left_caterpillar_signal_base_forward, config.back_left_caterpillar_signal_base_backward,
          CommHandlerControl.TTControlParamsFrameType.TTCONTROL_BACK_LEFT_CATERPILLAR_SIGNAL_BASE_TYPE);
    }
    if (level & TTControl_param_flags.TTCONTROL_BACK_RIGHT_CATERPILLAR_PID_PARAMS_FLAG)
    {
      CommHandlerControl.vSendCaterpillarPIDParameters(
          config.back_right_caterpillar_kp, config.back_right_caterpillar_ki, config.back_right_caterpillar_kd,
          CommHandlerControl.TTControlParamsFrameType.TTCONTROL_BACK_RIGHT_CATERPILLAR_PID_PARAMS_TYPE);
    }
    if (level & TTControl_param_flags.TTCONTROL_BACK_RIGHT_CATERPILLAR_SIGNALS_BASE_FLAG)
    {
      CommHandlerControl.vSendCaterpillarBaseSignal(
          config.back_right_caterpillar_signal_base_forward, config.back_right_caterpillar_signal_base_backward,
          CommHandlerControl.TTControlParamsFrameType.TTCONTROL_BACK_RIGHT_CATERPILLAR_SIGNAL_BASE_TYPE);
    }
  }
}

void frontCallback(const agribot_msgs::AgribotCaterpillar& msg)
{
  CommHandlerControl.vSendVelocities(msg.front_linear, msg.front_angular, msg.back_linear, msg.back_angular);
}

void ttcontrol_connection_state(const std_msgs::String& msg)
{
  if (msg.data == TTCONTROL_CONNECTION_MSG)
  {
    CommHandlerControl.vSetTTControlConnectionStatus(TTCONTROL_CONNECTED);
  }
  else
  {
    CommHandlerControl.vSetTTControlConnectionStatus(TTCONTROL_DISCONNECTED);
  }
}

bool read_encoders_data_file()
{
  if (!CommHandlerControl.bReadCartsEncodersDataFile(READ_REGULAR_CARTS_ENCODERS_FILE))
  {
    return CommHandlerControl.bReadCartsEncodersDataFile(READ_BACKUP_CARTS_ENCODERS_FILE);
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "agribot_control");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Subscriber sub = n.subscribe("/agribot/control/caterpillar", 1000, frontCallback);
  ros::Subscriber sub_connection_state =
      n.subscribe("/agribot/ttcontrol/connection_state", 1, ttcontrol_connection_state);
  ros::Publisher CartsStatusPublisher =
      n.advertise<agribot_msgs::AgribotCaterpillar>("/agribot/encoder/caterpillar", 1);
  ros::Publisher FrontCartAnglePublisher = n.advertise<std_msgs::Float32>("/agribot/encoder/cart/front", 1);
  ros::Publisher BackCartAnglePublisher = n.advertise<std_msgs::Float32>("/agribot/encoder/cart/back", 1);
  agribot_msgs::AgribotCaterpillar AgribotCaterpillarMsg;
  dynamic_reconfigure::Server<hal_agribot_control::TTControlParametersConfig> server;
  dynamic_reconfigure::Server<hal_agribot_control::TTControlParametersConfig>::CallbackType callback_function_object;
  std_msgs::Float32 FrontAngleMsg;
  std_msgs::Float32 BackAngleMsg;
  // angle initialization reading
  std::string path_to_carts_encoders_file;
  n.getParam("path_to_encoder_file", path_to_carts_encoders_file);
  CommHandlerControl.vSetPathToCartsEncodersDataFile(path_to_carts_encoders_file);
  if (!read_encoders_data_file())
  {
    ROS_ERROR_STREAM("[agribot_control_node] Failed to read carts angle data file content ! Robots carts encoders "
                     "values are not inited! \n"
                     << "Attempted path: " << path_to_carts_encoders_file);
    return -1;
  }
  CommHandlerControl.vSendCartsEncodersInitialization();
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
      CartsStatus cartStatus = CommHandlerControl.getCartsStatus();
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
      ROS_INFO("[agribot_control_node] Received automatic mode request");
      usleep(WAIT_FOR_TTCONTROL);
      CommHandlerControl.vConfirmAdvantechReadiness();
    }
    // if automatic state request error frame received in u8ParseCanFrames()
    // method
    if ((u8Status & AUTOMATIC_STATE_REQUEST_ERROR_MASK) == AUTOMATIC_STATE_REQUEST_ERROR_MASK)
    {
      ROS_ERROR("[agribot_control_node] Automatic state request error: %d", CommHandlerControl.iGetErrorCode());
    }
    // when connected, read and send encoders value initialization till the confirmation
    if (!CommHandlerControl.bGetEncodersConfirmedStatus() && CommHandlerControl.bGetTTControlConnectionStatus())
    {
      if (!read_encoders_data_file())
      {
        ROS_ERROR_STREAM("[agribot_control_node] Failed to read carts angle data file content ! Robots carts encoders "
                         "values are not inited! \n"
                         << "Attempted path: " << path_to_carts_encoders_file);
        return -1;
      }
      CommHandlerControl.vSendCartsEncodersInitialization();
    }
    // check if the carts has been initialized and write the new received values to file
    if (CommHandlerControl.bGetEncodersConfirmedStatus())
    {
      // check if both angle data reached us
      if (!CommHandlerControl.bWriteEncodersDataFile())
      {
        ROS_ERROR("[agribot_control_node] Failed to write carts encoders data to file!");
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
