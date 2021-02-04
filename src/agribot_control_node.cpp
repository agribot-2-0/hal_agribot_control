/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */
#include <agribot_msgs/AgribotCaterpillar.h>
#include <agribot_msgs/ChangeDeltasGeometry.h>
#include <hal_ttcontrol_communication/TTCommControl.h>
#include <dynamic_reconfigure/server.h>
#include <hal_agribot_control/TTControlParametersConfig.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <string_view>
#include <cstdint>

using std::int16_t;
using std::int32_t;
using std::int8_t;
using std::uint16_t;
using std::uint32_t;
using std::uint8_t;

constexpr char used_can[] = "can0";
constexpr uint WAIT_FOR_TTCONTROL = 100000;
constexpr uint8_t CARTS_ENCODERS_INIT_TIMEOUT_LIMIT = 10;
constexpr bool READ_BACKUP_CARTS_ENCODERS_FILE = false;
constexpr bool READ_REGULAR_CARTS_ENCODERS_FILE = true;
constexpr std::string_view TTCONTROL_CONNECTION_MSG = "connected";
constexpr bool TTCONTROL_CONNECTED = true;
constexpr bool TTCONTROL_DISCONNECTED = false;

enum RobotsCarts : uint8_t
{
  front_cart = 1,
  back_cart = 0
};

enum TTControlParameterFlags : uint32_t
{
  ttcontrol_signal_params_flag = 0x01,
  ttcontrol_front_pid_t_params_flag = 0x02,
  ttcontrol_front_pid_r_params_flag = 0x04,
   ttcontrol_back_pid_t_params_flag = 0x08,
   ttcontrol_back_pid_r_params_flag = 0x10,
   ttcontrol_front_left_caterpillar_pid_params_flag = 0x20,
   ttcontrol_front_left_caterpillar_signals_base_flag = 0x40,
   ttcontrol_front_right_caterpillar_pid_params_flag = 0x80,
   ttcontrol_front_right_caterpillar_signals_base_flag = 0x100,
   ttcontrol_back_left_caterpillar_pid_params_flag = 0x200,
   ttcontrol_back_left_caterpillar_signals_base_flag = 0x400,
   ttcontrol_back_right_caterpillar_pid_params_flag = 0x800,
   ttcontrol_back_right_caterpillar_signals_base_flag = 0x1000
};

TTCommControl CommHandlerControl(used_can);

void callbackDynamicReconfigure(const hal_agribot_control::TTControlParametersConfig& config, uint32_t level)
{
  // maximum number of queed msg to read in TTControl for a given CAN_ID is 10, so we need to send the msgs
  // in turns
  if (level == 0xFFFFFFFF)
  {
    CommHandlerControl.sendSignalsParameters(config.ditter_period_int, config.ditter_amplitude_int,
                                             config.encoder_period_int);
    CommHandlerControl.sendCartsPidParameters(
        config.front_kp_t_double, config.front_ki_t_double, config.front_kd_t_double, config.front_acc_t_double,
        TTControlParamsType::ttcontrol_front_pid_t_params_type);
    CommHandlerControl.sendCartsPidParameters(
        config.front_kp_r_double, config.front_ki_r_double, config.front_kd_r_double, config.front_acc_r_double,
        TTControlParamsType::ttcontrol_front_pid_r_params_type);
    CommHandlerControl.sendCartsPidParameters(
        config.back_kp_t_double, config.back_ki_t_double, config.back_kd_t_double, config.back_acc_t_double,
        TTControlParamsType::ttcontrol_back_pid_t_params_type);
    CommHandlerControl.sendCartsPidParameters(
        config.back_kp_r_double, config.back_ki_r_double, config.back_kd_r_double, config.back_acc_r_double,
        TTControlParamsType::ttcontrol_back_pid_r_params_type);
    CommHandlerControl.sendCaterpillarPidParameters(
        config.front_left_caterpillar_kp, config.front_left_caterpillar_ki, config.front_left_caterpillar_kd,
        TTControlParamsType::ttcontrol_front_left_caterpillar_pid_params_type);

    ros::Duration(0.5).sleep();  // sleep for half a second, to give TTControl time to read and save the parameters
    CommHandlerControl.sendCaterpillarBaseSignal(
        config.front_left_caterpillar_signal_base_forward, config.front_left_caterpillar_signal_base_backward,
        TTControlParamsType::ttcontrol_front_left_caterpillar_signal_base_type);
    CommHandlerControl.sendCaterpillarPidParameters(
        config.front_right_caterpillar_kp, config.front_right_caterpillar_ki, config.front_right_caterpillar_kd,
        TTControlParamsType::ttcontrol_front_right_caterpillar_pid_params_type);
    CommHandlerControl.sendCaterpillarBaseSignal(
        config.front_right_caterpillar_signal_base_forward, config.front_right_caterpillar_signal_base_backward,
        TTControlParamsType::ttcontrol_front_right_caterpillar_signal_base_type);
    CommHandlerControl.sendCaterpillarPidParameters(
        config.back_left_caterpillar_kp, config.back_left_caterpillar_ki, config.back_left_caterpillar_kd,
        TTControlParamsType::ttcontrol_back_left_caterpillar_pid_params_type);
    CommHandlerControl.sendCaterpillarBaseSignal(
        config.back_left_caterpillar_signal_base_forward, config.back_left_caterpillar_signal_base_backward,
        TTControlParamsType::ttcontrol_back_left_caterpillar_signal_base_type);
    CommHandlerControl.sendCaterpillarPidParameters(
        config.back_right_caterpillar_kp, config.back_right_caterpillar_ki, config.back_right_caterpillar_kd,
        TTControlParamsType::ttcontrol_back_right_caterpillar_pid_params_type);
    CommHandlerControl.sendCaterpillarBaseSignal(
        config.back_right_caterpillar_signal_base_forward, config.back_right_caterpillar_signal_base_backward,
        TTControlParamsType::ttcontrol_back_right_caterpillar_signal_base_type);
  }
  else
  {
    if (level & TTControlParameterFlags::ttcontrol_signal_params_flag)
    {
      CommHandlerControl.sendSignalsParameters(config.ditter_period_int, config.ditter_amplitude_int,
                                               config.encoder_period_int);
    }
    if (level & TTControlParameterFlags::ttcontrol_front_pid_t_params_flag)
    {
      CommHandlerControl.sendCartsPidParameters(
          config.front_kp_t_double, config.front_ki_t_double, config.front_kd_t_double, config.front_acc_t_double,
          TTControlParamsType::ttcontrol_front_pid_t_params_type);
    }
    if (level & TTControlParameterFlags::ttcontrol_front_pid_r_params_flag)
    {
      CommHandlerControl.sendCartsPidParameters(
          config.front_kp_r_double, config.front_ki_r_double, config.front_kd_r_double, config.front_acc_r_double,
          TTControlParamsType::ttcontrol_front_pid_r_params_type);
    }
    if (level & TTControlParameterFlags::ttcontrol_back_pid_t_params_flag)
    {
      CommHandlerControl.sendCartsPidParameters(
          config.back_kp_t_double, config.back_ki_t_double, config.back_kd_t_double, config.back_acc_t_double,
          TTControlParamsType::ttcontrol_back_pid_t_params_type);
    }
    if (level & TTControlParameterFlags::ttcontrol_back_pid_r_params_flag)
    {
      CommHandlerControl.sendCartsPidParameters(
          config.back_kp_r_double, config.back_ki_r_double, config.back_kd_r_double, config.back_acc_r_double,
          TTControlParamsType::ttcontrol_back_pid_r_params_type);
    }
    if (level & TTControlParameterFlags::ttcontrol_front_left_caterpillar_pid_params_flag)
    {
      CommHandlerControl.sendCaterpillarPidParameters(
          config.front_left_caterpillar_kp, config.front_left_caterpillar_ki, config.front_left_caterpillar_kd,
          TTControlParamsType::ttcontrol_front_left_caterpillar_pid_params_type);
    }
    if (level & TTControlParameterFlags::ttcontrol_front_left_caterpillar_signals_base_flag)
    {
      CommHandlerControl.sendCaterpillarBaseSignal(
          config.front_left_caterpillar_signal_base_forward, config.front_left_caterpillar_signal_base_backward,
          TTControlParamsType::ttcontrol_front_left_caterpillar_signal_base_type);
    }
    if (level & TTControlParameterFlags::ttcontrol_front_right_caterpillar_pid_params_flag)
    {
      CommHandlerControl.sendCaterpillarPidParameters(
          config.front_right_caterpillar_kp, config.front_right_caterpillar_ki, config.front_right_caterpillar_kd,
          TTControlParamsType::ttcontrol_front_right_caterpillar_pid_params_type);
    }
    if (level & TTControlParameterFlags::ttcontrol_front_right_caterpillar_signals_base_flag)
    {
      CommHandlerControl.sendCaterpillarBaseSignal(
          config.front_right_caterpillar_signal_base_forward, config.front_right_caterpillar_signal_base_backward,
          TTControlParamsType::ttcontrol_front_right_caterpillar_signal_base_type);
    }
    if (level & TTControlParameterFlags::ttcontrol_back_left_caterpillar_pid_params_flag)
    {
      CommHandlerControl.sendCaterpillarPidParameters(
          config.back_left_caterpillar_kp, config.back_left_caterpillar_ki, config.back_left_caterpillar_kd,
          TTControlParamsType::ttcontrol_back_left_caterpillar_pid_params_type);
    }
    if (level & TTControlParameterFlags::ttcontrol_back_left_caterpillar_signals_base_flag)
    {
      CommHandlerControl.sendCaterpillarBaseSignal(
          config.back_left_caterpillar_signal_base_forward, config.back_left_caterpillar_signal_base_backward,
          TTControlParamsType::ttcontrol_back_left_caterpillar_signal_base_type);
    }
    if (level & TTControlParameterFlags::ttcontrol_back_right_caterpillar_pid_params_flag)
    {
      CommHandlerControl.sendCaterpillarPidParameters(
          config.back_right_caterpillar_kp, config.back_right_caterpillar_ki, config.back_right_caterpillar_kd,
          TTControlParamsType::ttcontrol_back_right_caterpillar_pid_params_type);
    }
    if (level & TTControlParameterFlags::ttcontrol_back_right_caterpillar_signals_base_flag)
    {
      CommHandlerControl.sendCaterpillarBaseSignal(
          config.back_right_caterpillar_signal_base_forward, config.back_right_caterpillar_signal_base_backward,
          TTControlParamsType::ttcontrol_back_right_caterpillar_signal_base_type);
    }
  }
}

void frontCallback(const agribot_msgs::AgribotCaterpillar& msg)
{
  CommHandlerControl.sendVelocities(msg.front_linear, msg.front_angular, msg.back_linear, msg.back_angular);
}

bool sendDeltasGeometryCommand(const RobotsCarts selected_cart, const agribot_msgs::ChangeDeltasGeometry::Request& req)
{
  if (req.cart_delta_command == agribot_msgs::ChangeDeltasGeometry::Request::ELEVATED_CATERPILLAR_GEOMETRY ||
      req.cart_delta_command == agribot_msgs::ChangeDeltasGeometry::Request::NORMAL_CATERPILLAR_GEOMETRY)
  {
    if (selected_cart == RobotsCarts::front_cart)
    {
      CommHandlerControl.sendDeltasGeometryCommand(
          req.cart_delta_command, agribot_msgs::ChangeDeltasGeometry::Request::UNKNOWN_CATERPILLAR_GEOMETRY);
      return true;
    }
    else
    {
      CommHandlerControl.sendDeltasGeometryCommand(
          agribot_msgs::ChangeDeltasGeometry::Request::UNKNOWN_CATERPILLAR_GEOMETRY, req.cart_delta_command);
      return true;
    }
  }
  else
  {
    return false;
  }
}

void waitTillDeltasGeometryIsSet(const RobotsCarts selected_cart,
                                 const agribot_msgs::ChangeDeltasGeometry::Request& req)
{
  const CartsStatus& cartsStatus = CommHandlerControl.getCartsStatus();
  uint8_t cart_state = agribot_msgs::ChangeDeltasGeometry::Response::UNKNOWN_CATERPILLAR_GEOMETRY;
  if (selected_cart == RobotsCarts::front_cart)
  {
    while (cart_state != req.cart_delta_command)
    {
      cart_state = cartsStatus.getFrontCartGeometryState();
      ros::Duration(0.1).sleep();
    }
  }
  else
  {
    while (cart_state != req.cart_delta_command)
    {
      cart_state = cartsStatus.getBackCartGeometryState();
      ros::Duration(0.1).sleep();
    }
  }
}

void setDeltasGeometryResult(agribot_msgs::ChangeDeltasGeometry::Response& res,  // NOLINT(runtime/references)
                             const agribot_msgs::ChangeDeltasGeometry::Request& req)
{
  switch (req.cart_delta_command)
  {
    case agribot_msgs::ChangeDeltasGeometry::Request::ELEVATED_CATERPILLAR_GEOMETRY:
      res.result = res.ELEVATED_CATERPILLAR_GEOMETRY;
      break;
    case agribot_msgs::ChangeDeltasGeometry::Request::NORMAL_CATERPILLAR_GEOMETRY:
      res.result = res.NORMAL_CATERPILLAR_GEOMETRY;
      break;
    default:
      res.result = res.UNKNOWN_CATERPILLAR_GEOMETRY;
      break;
  }
}

bool deltasChangeGeometryServiceExecution(
    const agribot_msgs::ChangeDeltasGeometry::Request& req,
    agribot_msgs::ChangeDeltasGeometry::Response& res,  // NOLINT(runtime/references)
    const RobotsCarts selected_cart)
{
  if (sendDeltasGeometryCommand(selected_cart, req))
  {
    waitTillDeltasGeometryIsSet(selected_cart, req);
    setDeltasGeometryResult(res, req);
    return true;
  }
  else
  {
    res.result = res.UNKNOWN_CATERPILLAR_GEOMETRY;
    return false;
  }
}

bool frontDeltasGeometryServiceCallback(
    agribot_msgs::ChangeDeltasGeometry::Request& req,   // NOLINT(runtime/references)
    agribot_msgs::ChangeDeltasGeometry::Response& res)  // NOLINT(runtime/references)
{
  return deltasChangeGeometryServiceExecution(req, res, RobotsCarts::front_cart);
}

bool backDeltasGeometryServiceCallback(agribot_msgs::ChangeDeltasGeometry::Request& req,   // NOLINT(runtime/references)
                                       agribot_msgs::ChangeDeltasGeometry::Response& res)  // NOLINT(runtime/references)
{
  return deltasChangeGeometryServiceExecution(req, res, RobotsCarts::back_cart);
}

void ttcontrolConnectionState(const std_msgs::String& msg)
{
  if (msg.data == TTCONTROL_CONNECTION_MSG)
  {
    CommHandlerControl.setTtcontrolConnectionStatus(TTCONTROL_CONNECTED);
  }
  else
  {
    CommHandlerControl.setTtcontrolConnectionStatus(TTCONTROL_DISCONNECTED);
  }
}

bool readEncodersDataFile()
{
  if (!CommHandlerControl.readCartsEncodersDataFile(READ_REGULAR_CARTS_ENCODERS_FILE))
  {
    return CommHandlerControl.readCartsEncodersDataFile(READ_BACKUP_CARTS_ENCODERS_FILE);
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "agribot_control");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::AsyncSpinner spinner(2);  // Use 2 threads
  spinner.start();

  ros::Subscriber sub = n.subscribe("/agribot/control/caterpillar", 1000, frontCallback);
  ros::Subscriber sub_connection_state =
      n.subscribe("/agribot/ttcontrol/connection_state", 1, ttcontrolConnectionState);
  ros::Publisher CartsStatusPublisher =
      n.advertise<agribot_msgs::AgribotCaterpillar>("/agribot/encoder/caterpillar", 1);
  ros::Publisher FrontCartAnglePublisher = n.advertise<std_msgs::Float32>("/agribot/encoder/cart/front", 1);
  ros::Publisher BackCartAnglePublisher = n.advertise<std_msgs::Float32>("/agribot/encoder/cart/back", 1);
  agribot_msgs::AgribotCaterpillar AgribotCaterpillarMsg;
  dynamic_reconfigure::Server<hal_agribot_control::TTControlParametersConfig> server;
  dynamic_reconfigure::Server<hal_agribot_control::TTControlParametersConfig>::CallbackType callback_function_object;
  ros::ServiceServer service_front =
      n.advertiseService("agribot/cart/caterpillar/front/change_deltas_geometry", frontDeltasGeometryServiceCallback);
  ros::ServiceServer service_back =
      n.advertiseService("agribot/cart/caterpillar/back/change_deltas_geometry", backDeltasGeometryServiceCallback);
  std_msgs::Float32 FrontAngleMsg;
  std_msgs::Float32 BackAngleMsg;
  // angle initialization reading
  std::string path_to_carts_encoders_file;
  n.getParam("path_to_encoder_file", path_to_carts_encoders_file);
  CommHandlerControl.setPathToCartsEncodersDataFile(path_to_carts_encoders_file);
  if (!readEncodersDataFile())
  {
    ROS_ERROR_STREAM("[AgribotControlNode] Failed to read carts angle data file content ! Robots carts encoders "
                     "values are not inited! \n"
                     << "Attempted path: " << path_to_carts_encoders_file);
    return -1;
  }
  CommHandlerControl.sendCartsEncodersInitialization();
  callback_function_object = boost::bind(&callbackDynamicReconfigure, _1, _2);
  server.setCallback(callback_function_object);
  uint8_t encoder_init_timeout_counter = 0;
  while (ros::ok())
  {
    CommHandlerControl.receiveCanFrames();
    CommHandlerControl.parseCanFrames();
    if (CommHandlerControl.checkReceivedCanBusIdBit(VELOCITIES_MASK))
    {
      const CartsStatus& cartStatus = CommHandlerControl.getCartsStatus();
      AgribotCaterpillarMsg.header.stamp = ros::Time::now();
      AgribotCaterpillarMsg.header.frame_id = "hal_caterpillar_feedback";
      AgribotCaterpillarMsg.front_linear = cartStatus.getFrontLinearVelocity();
      AgribotCaterpillarMsg.front_angular = cartStatus.getFrontAngularVelocity();
      AgribotCaterpillarMsg.back_linear = cartStatus.getBackLinearVelocity();
      AgribotCaterpillarMsg.back_angular = cartStatus.getBackAngularVelocity();
      CartsStatusPublisher.publish(AgribotCaterpillarMsg);
      CommHandlerControl.clearReceivedCanBusIdBit(VELOCITIES_MASK);
    }
    if (CommHandlerControl.checkReceivedCanBusIdBit(FRONT_ANGLE_MASK))
    {
      FrontAngleMsg.data = CommHandlerControl.getFrontCartAngle();
      FrontCartAnglePublisher.publish(FrontAngleMsg);
      CommHandlerControl.clearReceivedCanBusIdBit(FRONT_ANGLE_MASK);
    }
    if (CommHandlerControl.checkReceivedCanBusIdBit(BACK_ANGLE_MASK))
    {
      BackAngleMsg.data = CommHandlerControl.getBackCartAngle();
      BackCartAnglePublisher.publish(BackAngleMsg);
      CommHandlerControl.clearReceivedCanBusIdBit(BACK_ANGLE_MASK);
    }
    if (CommHandlerControl.checkReceivedCanBusIdBit(AUTOMATIC_STATE_REQUEST_MASK))
    {
      // TODO(Michal/Adam): Implement some diagnostics
      ROS_INFO("[AgribotControlNode] Received automatic mode request");
      usleep(WAIT_FOR_TTCONTROL);
      CommHandlerControl.confirmAdvantechReadiness();
      CommHandlerControl.clearReceivedCanBusIdBit(AUTOMATIC_STATE_REQUEST_MASK);
    }
    if (CommHandlerControl.checkReceivedCanBusIdBit(AUTOMATIC_STATE_REQUEST_ERROR_MASK))
    {
      ROS_ERROR("[AgribotControlNode] Automatic state request error: %d", CommHandlerControl.getErrorCode());
      CommHandlerControl.clearReceivedCanBusIdBit(AUTOMATIC_STATE_REQUEST_ERROR_MASK);
    }
    // when connected, read and send encoders value initialization till the confirmation
    if (!CommHandlerControl.getEncodersConfirmedStatus() && CommHandlerControl.getTtcontrolConnectionStatus())
    {
      if (!readEncodersDataFile())
      {
        ROS_ERROR_STREAM("[AgribotControlNode] Failed to read carts angle data file content ! Robots carts encoders "
                         "values are not inited! \n"
                         << "Attempted path: " << path_to_carts_encoders_file);
        return -1;
      }
      CommHandlerControl.sendCartsEncodersInitialization();
    }
    // check if the carts has been initialized and write the new received values to file
    if (CommHandlerControl.getEncodersConfirmedStatus())
    {
      // check if both angle data reached us
      encoder_init_timeout_counter = 0;
      if (!CommHandlerControl.writeEncodersDataFile())
      {
        ROS_ERROR("[AgribotControlNode] Failed to write carts encoders data to file!");
      }
    }
    else
    {
      if (encoder_init_timeout_counter >= CARTS_ENCODERS_INIT_TIMEOUT_LIMIT)
      {
        ROS_ERROR("[AgribotControlNode] Carts encoders not inited!");
        encoder_init_timeout_counter = 0;
      }
      else
      {
        encoder_init_timeout_counter++;
      }
    }
    loop_rate.sleep();
  }
}
