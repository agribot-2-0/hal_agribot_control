/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */
#include <agribot_msgs/AgribotCaterpillar.h>
#include <agribot_msgs/AgribotCaterpillarWheelsRotation.h>
#include <agribot_msgs/ChangeDeltasGeometry.h>
#include <agribot_msgs/AgribotTool.h>
#include <hal_ttcontrol_communication/TTCommControl.h>
#include <hal_ttcontrol_communication/TplControl.h>
#include <dynamic_reconfigure/server.h>
#include <hal_agribot_control/TTControlParametersConfig.h>
#include <agribot_common/CommonFunctions/ros_common_functions.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <string_view>
#include <cstdint>

using std::int16_t;
using std::int32_t;
using std::int8_t;
using std::uint16_t;
using std::uint32_t;
using std::uint8_t;

constexpr uint WAIT_FOR_TTCONTROL = 100000;
constexpr uint8_t CARTS_ENCODERS_INIT_TIMEOUT_LIMIT = 10;
constexpr uint8_t ENGINE_SERVICE_TIMEOUT_LIMIT = 60;
constexpr bool READ_BACKUP_CARTS_ENCODERS_FILE = false;
constexpr bool READ_REGULAR_CARTS_ENCODERS_FILE = true;
constexpr std::string_view TTCONTROL_CONNECTION_MSG = "connected";
constexpr bool TTCONTROL_CONNECTED = true;
constexpr bool TTCONTROL_DISCONNECTED = false;

TTCommControl ttcontrol_communication_handler;

enum RobotsCarts : uint8_t
{
  front_cart = 1,
  back_cart = 0
};

enum TTControlParameterFlag : uint32_t
{
  signal_params = 0x01,
  front_pid_t_params = 0x02,
  front_pid_r_params = 0x04,
  back_pid_t_params = 0x08,
  back_pid_r_params = 0x10,
  front_left_caterpillar_pid_params = 0x20,
  front_left_caterpillar_signals_base = 0x40,
  front_right_caterpillar_pid_params = 0x80,
  front_right_caterpillar_signals_base = 0x100,
  back_left_caterpillar_pid_params = 0x200,
  back_left_caterpillar_signals_base = 0x400,
  back_right_caterpillar_pid_params = 0x800,
  back_right_caterpillar_signals_base = 0x1000,
  override_speed_and_valve_limits_params = 0x2000,
  stabilization_params = 0x4000,
  oil_and_engine_params = 0x8000,
  every_flag_set = 0xFFFFFFFF
};

TplControl tpl_control_msg_to_tpl_control_struct(const agribot_msgs::AgribotTool& msg)
{
  TplControl control_values;
  control_values.angle_actuator = static_cast<std::int16_t>(msg.tpl_angle);
  control_values.height_actuator = static_cast<std::int16_t>(msg.tpl_height);
  control_values.red_qcf_valve = static_cast<std::int16_t>(msg.red_valve_flow);
  control_values.green_qcf_valve = static_cast<std::int16_t>(msg.green_valve_flow);
  control_values.blue_qcf_valve = static_cast<std::int16_t>(msg.blue_valve_flow);
  control_values.pto_valve = static_cast<std::int16_t>(msg.pto_rpm);
  return control_values;
}

TtcontrolOverrideParameters&&
parseConfigToOverrideParameters(const hal_agribot_control::TTControlParametersConfig& config)
{
  TtcontrolOverrideParameters override_params_data;
  override_params_data.cart_rotation_limit = config.cart_rotation_limit;
  override_params_data.override_velocity_linear = config.override_velocity_linear;
  override_params_data.override_velocity_angular = config.override_velocity_angular;
  override_params_data.override_valves_limits = config.override_valves_limits;
  override_params_data.deltas_geometry_valves_push_limit_front = config.deltas_geometry_valves_push_limit_front;
  override_params_data.deltas_geometry_valves_push_limit_back = config.deltas_geometry_valves_push_limit_back;
  override_params_data.deltas_geometry_valves_pull_front_back = config.deltas_geometry_valves_pull_front_back;
  return std::move(override_params_data);
}

void callbackDynamicReconfigure(const hal_agribot_control::TTControlParametersConfig& config, uint32_t level)
{
  // maximum number of queed msg to read in TTControl for a given CAN_ID is 10, so we need to send the msgs
  // in turns
  if (level == TTControlParameterFlag::every_flag_set)
  {
    ttcontrol_communication_handler.sendSignalsParameters(config.ditter_period_int, config.ditter_amplitude_int,
                                                          config.encoder_period_int);
    ttcontrol_communication_handler.sendCartsPidParameters(config.front_kp_t_double, config.front_ki_t_double,
                                                           config.front_kd_t_double, config.front_acc_t_double,
                                                           TTControlParameterType::front_pid_t);
    ttcontrol_communication_handler.sendCartsPidParameters(config.front_kp_r_double, config.front_ki_r_double,
                                                           config.front_kd_r_double, config.front_acc_r_double,
                                                           TTControlParameterType::front_pid_r);
    ttcontrol_communication_handler.sendCartsPidParameters(config.back_kp_t_double, config.back_ki_t_double,
                                                           config.back_kd_t_double, config.back_acc_t_double,
                                                           TTControlParameterType::back_pid_t);
    ttcontrol_communication_handler.sendCartsPidParameters(config.back_kp_r_double, config.back_ki_r_double,
                                                           config.back_kd_r_double, config.back_acc_r_double,
                                                           TTControlParameterType::back_pid_r);
    ttcontrol_communication_handler.sendCaterpillarPidParameters(
        config.front_left_caterpillar_kp, config.front_left_caterpillar_ki, config.front_left_caterpillar_kd,
        TTControlParameterType::front_left_caterpillar_pid);

    ros::Duration(0.5).sleep();  // sleep for half a second, to give TTControl time to read and save the parameters
    ttcontrol_communication_handler.sendCaterpillarBaseSignal(
        config.front_left_caterpillar_signal_base_forward, config.front_left_caterpillar_signal_base_backward,
        TTControlParameterType::front_left_caterpillar_signal_base);
    ttcontrol_communication_handler.sendCaterpillarPidParameters(
        config.front_right_caterpillar_kp, config.front_right_caterpillar_ki, config.front_right_caterpillar_kd,
        TTControlParameterType::front_right_caterpillar_pid);
    ttcontrol_communication_handler.sendCaterpillarBaseSignal(
        config.front_right_caterpillar_signal_base_forward, config.front_right_caterpillar_signal_base_backward,
        TTControlParameterType::front_right_caterpillar_signal_base);
    ttcontrol_communication_handler.sendCaterpillarPidParameters(
        config.back_left_caterpillar_kp, config.back_left_caterpillar_ki, config.back_left_caterpillar_kd,
        TTControlParameterType::back_left_caterpillar_pid);
    ttcontrol_communication_handler.sendCaterpillarBaseSignal(
        config.back_left_caterpillar_signal_base_forward, config.back_left_caterpillar_signal_base_backward,
        TTControlParameterType::back_left_caterpillar_signal_base);
    ttcontrol_communication_handler.sendCaterpillarPidParameters(
        config.back_right_caterpillar_kp, config.back_right_caterpillar_ki, config.back_right_caterpillar_kd,
        TTControlParameterType::back_right_caterpillar_pid);
    ttcontrol_communication_handler.sendCaterpillarBaseSignal(
        config.back_right_caterpillar_signal_base_forward, config.back_right_caterpillar_signal_base_backward,
        TTControlParameterType::back_right_caterpillar_signal_base);

    ros::Duration(0.5).sleep();  // sleep for half a second, to give TTControl time to read and save the parameters
    ttcontrol_communication_handler.sendOverrideLimits(parseConfigToOverrideParameters(config));
    ttcontrol_communication_handler.sendStabilizationParameters(config.stab_pid_kp, config.stab_pid_kd,
                                                                config.stab_pid_ki, config.stab_pid_max_signal);
    ttcontrol_communication_handler.sendOilAndEngineParams(config.max_oil_temperature, config.min_oil_temperature,
                                                           config.histeresis_oil_temperature, config.engine_rpm_max,
                                                           config.engine_rpm_min);
  }
  else
  {
    if (level & TTControlParameterFlag::signal_params)
    {
      ttcontrol_communication_handler.sendSignalsParameters(config.ditter_period_int, config.ditter_amplitude_int,
                                                            config.encoder_period_int);
    }
    if (level & TTControlParameterFlag::front_pid_t_params)
    {
      ttcontrol_communication_handler.sendCartsPidParameters(config.front_kp_t_double, config.front_ki_t_double,
                                                             config.front_kd_t_double, config.front_acc_t_double,
                                                             TTControlParameterType::front_pid_t);
    }
    if (level & TTControlParameterFlag::front_pid_r_params)
    {
      ttcontrol_communication_handler.sendCartsPidParameters(config.front_kp_r_double, config.front_ki_r_double,
                                                             config.front_kd_r_double, config.front_acc_r_double,
                                                             TTControlParameterType::front_pid_r);
    }
    if (level & TTControlParameterFlag::back_pid_t_params)
    {
      ttcontrol_communication_handler.sendCartsPidParameters(config.back_kp_t_double, config.back_ki_t_double,
                                                             config.back_kd_t_double, config.back_acc_t_double,
                                                             TTControlParameterType::back_pid_t);
    }
    if (level & TTControlParameterFlag::back_pid_r_params)
    {
      ttcontrol_communication_handler.sendCartsPidParameters(config.back_kp_r_double, config.back_ki_r_double,
                                                             config.back_kd_r_double, config.back_acc_r_double,
                                                             TTControlParameterType::back_pid_r);
    }
    if (level & TTControlParameterFlag::front_left_caterpillar_pid_params)
    {
      ttcontrol_communication_handler.sendCaterpillarPidParameters(
          config.front_left_caterpillar_kp, config.front_left_caterpillar_ki, config.front_left_caterpillar_kd,
          TTControlParameterType::front_left_caterpillar_pid);
    }
    if (level & TTControlParameterFlag::front_left_caterpillar_signals_base)
    {
      ttcontrol_communication_handler.sendCaterpillarBaseSignal(
          config.front_left_caterpillar_signal_base_forward, config.front_left_caterpillar_signal_base_backward,
          TTControlParameterType::front_left_caterpillar_signal_base);
    }
    if (level & TTControlParameterFlag::front_right_caterpillar_pid_params)
    {
      ttcontrol_communication_handler.sendCaterpillarPidParameters(
          config.front_right_caterpillar_kp, config.front_right_caterpillar_ki, config.front_right_caterpillar_kd,
          TTControlParameterType::front_right_caterpillar_pid);
    }
    if (level & TTControlParameterFlag::front_right_caterpillar_signals_base)
    {
      ttcontrol_communication_handler.sendCaterpillarBaseSignal(
          config.front_right_caterpillar_signal_base_forward, config.front_right_caterpillar_signal_base_backward,
          TTControlParameterType::front_right_caterpillar_signal_base);
    }
    if (level & TTControlParameterFlag::back_left_caterpillar_pid_params)
    {
      ttcontrol_communication_handler.sendCaterpillarPidParameters(
          config.back_left_caterpillar_kp, config.back_left_caterpillar_ki, config.back_left_caterpillar_kd,
          TTControlParameterType::back_left_caterpillar_pid);
    }
    if (level & TTControlParameterFlag::back_left_caterpillar_signals_base)
    {
      ttcontrol_communication_handler.sendCaterpillarBaseSignal(
          config.back_left_caterpillar_signal_base_forward, config.back_left_caterpillar_signal_base_backward,
          TTControlParameterType::back_left_caterpillar_signal_base);
    }
    if (level & TTControlParameterFlag::back_right_caterpillar_pid_params)
    {
      ttcontrol_communication_handler.sendCaterpillarPidParameters(
          config.back_right_caterpillar_kp, config.back_right_caterpillar_ki, config.back_right_caterpillar_kd,
          TTControlParameterType::back_right_caterpillar_pid);
    }
    if (level & TTControlParameterFlag::back_right_caterpillar_signals_base)
    {
      ttcontrol_communication_handler.sendCaterpillarBaseSignal(
          config.back_right_caterpillar_signal_base_forward, config.back_right_caterpillar_signal_base_backward,
          TTControlParameterType::back_right_caterpillar_signal_base);
    }
    if (level & TTControlParameterFlag::override_speed_and_valve_limits_params)
    {
      ttcontrol_communication_handler.sendOverrideLimits(parseConfigToOverrideParameters(config));
    }
    if (level & TTControlParameterFlag::stabilization_params)
    {
      ttcontrol_communication_handler.sendStabilizationParameters(config.stab_pid_kp, config.stab_pid_kd,
                                                                  config.stab_pid_ki, config.stab_pid_max_signal);
    }
    if (level & TTControlParameterFlag::oil_and_engine_params)
    {
      ttcontrol_communication_handler.sendOilAndEngineParams(config.max_oil_temperature, config.min_oil_temperature,
                                                             config.histeresis_oil_temperature, config.engine_rpm_max,
                                                             config.engine_rpm_min);
    }
  }
}

void caterpillarsControlCallback(const agribot_msgs::AgribotCaterpillar& msg)
{
  ttcontrol_communication_handler.sendVelocities(msg.front_linear, msg.front_angular, msg.back_linear,
                                                 msg.back_angular);
}

void frontTplControlCallback(const agribot_msgs::AgribotTool& msg)
{
  ttcontrol_communication_handler.sendTplValveControlViaCan(SelectedTpl::front,
                                                            tpl_control_msg_to_tpl_control_struct(msg));
}

void backTplControlCallback(const agribot_msgs::AgribotTool& msg)
{
  ttcontrol_communication_handler.sendTplValveControlViaCan(SelectedTpl::back,
                                                            tpl_control_msg_to_tpl_control_struct(msg));
}

bool sendDeltasGeometryCommand(const RobotsCarts selected_cart, const agribot_msgs::ChangeDeltasGeometry::Request& req)
{
  if (req.cart_delta_command == agribot_msgs::ChangeDeltasGeometry::Request::ELEVATED_CATERPILLAR_GEOMETRY ||
      req.cart_delta_command == agribot_msgs::ChangeDeltasGeometry::Request::NORMAL_CATERPILLAR_GEOMETRY)
  {
    if (selected_cart == RobotsCarts::front_cart)
    {
      ttcontrol_communication_handler.sendDeltasGeometryCommand(
          req.cart_delta_command, agribot_msgs::ChangeDeltasGeometry::Request::UNKNOWN_CATERPILLAR_GEOMETRY);
      return true;
    }
    else
    {
      ttcontrol_communication_handler.sendDeltasGeometryCommand(
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
  const CartsStatus& cartsStatus = ttcontrol_communication_handler.getCartsStatus();
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

bool engineStartService(std_srvs::Trigger::Request& req,   // NOLINT(runtime/references)
                        std_srvs::Trigger::Response& res)  // NOLINT(runtime/references)
{
  static_cast<void>(req);
  ttcontrol_communication_handler.sendEngineCommand(TTControlEngineCommands::start_engine);
  // wait till the engine is started
  uint8_t timeout_counter = 0;
  while (!ttcontrol_communication_handler.getEngineStatus())
  {
    ros::Duration(0.1).sleep();
    timeout_counter++;
    if (timeout_counter > ENGINE_SERVICE_TIMEOUT_LIMIT)
    {
      // send the msg to stop engine to prevent its sudden start when resolving problems causing the failure
      ttcontrol_communication_handler.sendEngineCommand(TTControlEngineCommands::stop_engine);
      res.success = false;
      res.message = "Couldnt start engine, please check: \n- voltage level";
      res.message += "\n- fuel level \n- can interface connection";
      return false;
    }
  }
  res.message = "Engine started";
  res.success = true;
  return true;
}

bool engineStopService(std_srvs::Trigger::Request& req,   // NOLINT(runtime/references)
                       std_srvs::Trigger::Response& res)  // NOLINT(runtime/references)
{
  static_cast<void>(req);
  ttcontrol_communication_handler.sendEngineCommand(TTControlEngineCommands::stop_engine);
  // wait till the engine is stopped
  uint8_t timeout_counter = 0;
  while (ttcontrol_communication_handler.getEngineStatus())
  {
    ros::Duration(0.1).sleep();
    timeout_counter++;
    if (timeout_counter > ENGINE_SERVICE_TIMEOUT_LIMIT)
    {
      res.success = false;
      res.message = "Couldnt stop engine, please check: \n- can interface connection";
      return false;
    }
  }
  res.success = true;
  res.message = "Engine stopped";
  return true;
}

void ttcontrolConnectionState(const std_msgs::String& msg)
{
  if (msg.data == TTCONTROL_CONNECTION_MSG)
  {
    ttcontrol_communication_handler.setTtcontrolConnectionStatus(TTCONTROL_CONNECTED);
  }
  else
  {
    ttcontrol_communication_handler.setTtcontrolConnectionStatus(TTCONTROL_DISCONNECTED);
  }
}

bool readEncodersDataFile()
{
  if (!ttcontrol_communication_handler.readCartsEncodersDataFile(READ_REGULAR_CARTS_ENCODERS_FILE))
  {
    return ttcontrol_communication_handler.readCartsEncodersDataFile(READ_BACKUP_CARTS_ENCODERS_FILE);
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "agribot_control");
  std::string selected_can;
  try
  {
    selected_can = ros_common_functions::getRosParam<std::string>("/hal_agribot_control/can_bus_name");
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM("[HAL_AGRIBOT_CONTROL] " << ex.what());
    ros::shutdown();
  }
  std::unique_ptr<Can> can_handler = std::make_unique<Can>();
  can_handler->iInitialize(selected_can.c_str());
  ttcontrol_communication_handler.initializeCanHandler(std::move(can_handler));
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::AsyncSpinner spinner(2);  // Use 2 threads
  spinner.start();

  ros::Subscriber sub = n.subscribe("/agribot/control/caterpillar", 1, caterpillarsControlCallback);
  ros::Subscriber sub_connection_state =
      n.subscribe("/agribot/ttcontrol/connection_state", 1, ttcontrolConnectionState);
  ros::Subscriber tpl_front_control_sub = n.subscribe("/agribot/control/tool/front/valves", 1, frontTplControlCallback);
  ros::Subscriber tpl_back_control_sub = n.subscribe("/agribot/control/tool/back/valves", 1, backTplControlCallback);
  ros::Publisher carts_status_publisher =
      n.advertise<agribot_msgs::AgribotCaterpillar>("/agribot/encoder/caterpillar", 1);
  ros::Publisher track_encoder_sum_publisher =
      n.advertise<agribot_msgs::AgribotCaterpillarWheelsRotation>("/agribot/encoder/caterpillar/rotations/impulses", 1);
  ros::Publisher front_cart_angle_publisher = n.advertise<std_msgs::Float32>("/agribot/encoder/cart/front", 1);
  ros::Publisher back_cart_angle_publisher = n.advertise<std_msgs::Float32>("/agribot/encoder/cart/back", 1);
  dynamic_reconfigure::Server<hal_agribot_control::TTControlParametersConfig> parameter_server;
  dynamic_reconfigure::Server<hal_agribot_control::TTControlParametersConfig>::CallbackType callback_function_object;
  ros::ServiceServer service_front =
      n.advertiseService("/agribot/cart/caterpillar/front/change_deltas_geometry", frontDeltasGeometryServiceCallback);
  ros::ServiceServer service_back =
      n.advertiseService("/agribot/cart/caterpillar/back/change_deltas_geometry", backDeltasGeometryServiceCallback);
  ros::ServiceServer service_engine_start = n.advertiseService("/agribot/engine_start", engineStartService);
  ros::ServiceServer service_engine_stop = n.advertiseService("/agribot/engine_stop", engineStopService);
  std_msgs::Float32 front_angle_msg;
  std_msgs::Float32 back_angle_msg;
  agribot_msgs::AgribotCaterpillar agribot_caterpillar_msg;
  agribot_msgs::AgribotCaterpillarWheelsRotation encoder_sum_msg;
  // encoder initialization reading
  std::string path_to_carts_encoders_file;
  n.getParam("path_to_encoder_file", path_to_carts_encoders_file);
  ttcontrol_communication_handler.setPathToCartsEncodersDataFile(path_to_carts_encoders_file);
  if (!readEncodersDataFile())
  {
    ROS_ERROR_STREAM("[AgribotControlNode] Failed to read carts angle data file content ! Robots carts encoders "
                     "values are not inited! \n"
                     << "Attempted path: " << path_to_carts_encoders_file);
    return -1;
  }
  ttcontrol_communication_handler.sendCartsEncodersInitialization();
  callback_function_object = boost::bind(&callbackDynamicReconfigure, _1, _2);
  parameter_server.setCallback(callback_function_object);
  uint8_t encoder_init_timeout_counter = 0;
  while (ros::ok())
  {
    ttcontrol_communication_handler.receiveCanFrames();
    ttcontrol_communication_handler.parseCanFrames();
    if (ttcontrol_communication_handler.checkReceivedCanBusIdBit(ReceivedFramesRegisterMask::velocities))
    {
      const CartsStatus& cart_status = ttcontrol_communication_handler.getCartsStatus();
      agribot_caterpillar_msg.header.stamp = ros::Time::now();
      agribot_caterpillar_msg.header.frame_id = "hal_caterpillar_feedback";
      agribot_caterpillar_msg.front_linear = cart_status.getFrontLinearVelocity();
      agribot_caterpillar_msg.front_angular = cart_status.getFrontAngularVelocity();
      agribot_caterpillar_msg.back_linear = cart_status.getBackLinearVelocity();
      agribot_caterpillar_msg.back_angular = cart_status.getBackAngularVelocity();
      carts_status_publisher.publish(agribot_caterpillar_msg);
      ttcontrol_communication_handler.clearReceivedCanBusIdBit(ReceivedFramesRegisterMask::velocities);
    }
    if (ttcontrol_communication_handler.checkReceivedCanBusIdBit(ReceivedFramesRegisterMask::front_angle))
    {
      front_angle_msg.data = ttcontrol_communication_handler.getFrontCartAngle();
      front_cart_angle_publisher.publish(front_angle_msg);
      ttcontrol_communication_handler.clearReceivedCanBusIdBit(ReceivedFramesRegisterMask::front_angle);
    }
    if (ttcontrol_communication_handler.checkReceivedCanBusIdBit(ReceivedFramesRegisterMask::back_angle))
    {
      back_angle_msg.data = ttcontrol_communication_handler.getBackCartAngle();
      back_cart_angle_publisher.publish(back_angle_msg);
      ttcontrol_communication_handler.clearReceivedCanBusIdBit(ReceivedFramesRegisterMask::back_angle);
    }
    if (ttcontrol_communication_handler.checkReceivedCanBusIdBit(ReceivedFramesRegisterMask::automatic_state_request))
    {
      // TODO(Michal/Adam): Implement some diagnostics
      ROS_INFO("[AgribotControlNode] Received automatic mode request");
      usleep(WAIT_FOR_TTCONTROL);
      ttcontrol_communication_handler.confirmAdvantechReadiness();
      ttcontrol_communication_handler.clearReceivedCanBusIdBit(ReceivedFramesRegisterMask::automatic_state_request);
    }
    if (ttcontrol_communication_handler.checkReceivedCanBusIdBit(ReceivedFramesRegisterMask::track_encoder_sum))
    {
      CartsStatus cart_status = ttcontrol_communication_handler.getCartsStatus();
      encoder_sum_msg.header.stamp = ros::Time::now();
      encoder_sum_msg.header.frame_id = "hal_tracks_encoders_sum_feedback";
      encoder_sum_msg.front_left = cart_status.getTrackEncoderSum(EncoderTrackId::front_left);
      encoder_sum_msg.front_right = cart_status.getTrackEncoderSum(EncoderTrackId::front_right);
      encoder_sum_msg.back_left = cart_status.getTrackEncoderSum(EncoderTrackId::back_left);
      encoder_sum_msg.back_right = cart_status.getTrackEncoderSum(EncoderTrackId::back_right);
      track_encoder_sum_publisher.publish(encoder_sum_msg);
      ttcontrol_communication_handler.clearReceivedCanBusIdBit(ReceivedFramesRegisterMask::track_encoder_sum);
    }
    if (ttcontrol_communication_handler.checkReceivedCanBusIdBit(
            ReceivedFramesRegisterMask::automatic_state_request_error))
    {
      ROS_ERROR("[AgribotControlNode] Automatic state request error: %d",
                ttcontrol_communication_handler.getErrorCode());
      ttcontrol_communication_handler.clearReceivedCanBusIdBit(
          ReceivedFramesRegisterMask::automatic_state_request_error);
    }
    // when connected, read and send encoders value initialization till the confirmation
    if (!ttcontrol_communication_handler.getEncodersConfirmedStatus() &&
        ttcontrol_communication_handler.getTtcontrolConnectionStatus())
    {
      if (!readEncodersDataFile())
      {
        ROS_ERROR_STREAM("[AgribotControlNode] Failed to read carts angle data file content ! Robots carts encoders "
                         "values are not inited! \n"
                         << "Attempted path: " << path_to_carts_encoders_file);
        return -1;
      }
      ttcontrol_communication_handler.sendCartsEncodersInitialization();
    }
    // check if the carts has been initialized and write the new received values to file
    if (ttcontrol_communication_handler.getEncodersConfirmedStatus())
    {
      // check if both angle data reached us
      encoder_init_timeout_counter = 0;
      if (!ttcontrol_communication_handler.writeEncodersDataFile())
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
