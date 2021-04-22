/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include <hal_ttcontrol_communication/AgribotControlRosWrapper.h>

using std::int16_t;
using std::int32_t;
using std::int8_t;
using std::uint16_t;
using std::uint32_t;
using std::uint64_t;
using std::uint8_t;

void AgribotControlRosWrapper::registerSubscribers()
{
  sub_caterpillar_ =
      nh_->subscribe("/agribot/control/caterpillar", 1, &AgribotControlRosWrapper::caterpillarsControlCallback, this);
  sub_connection_state_ = nh_->subscribe("/agribot/ttcontrol/connection_state", 1,
                                                         &AgribotControlRosWrapper::ttcontrolConnectionState, this);
  tpl_front_control_sub_ =
      nh_->subscribe("/agribot/control/tool/front/valves", 1, &AgribotControlRosWrapper::frontTplControlCallback, this);
  tpl_back_control_sub_ =
      nh_->subscribe("/agribot/control/tool/back/valves", 1, &AgribotControlRosWrapper::backTplControlCallback, this);
}

void AgribotControlRosWrapper::registerPublishers()
{
  carts_status_publisher_ =
      nh_->advertise<agribot_msgs::AgribotCaterpillar>("/agribot/encoder/caterpillar", 1);
   track_encoder_sum_publisher_ =
      nh_->advertise<agribot_msgs::AgribotCaterpillarWheelsRotation>("/agribot/encoder/caterpillar/rotations/impulses",
                                                                     1);
  front_cart_angle_publisher_ = nh_->advertise<std_msgs::Float32>("/agribot/encoder/cart/front", 1);
  back_cart_angle_publisher_ = nh_->advertise<std_msgs::Float32>("/agribot/encoder/cart/back", 1);
}

void AgribotControlRosWrapper::registerServices()
{
   service_front_ =
      nh_->advertiseService("/agribot/cart/caterpillar/front/change_deltas_geometry",
                            &AgribotControlRosWrapper::frontDeltasGeometryServiceCallback, this);
   service_back_ =
      nh_->advertiseService("/agribot/cart/caterpillar/back/change_deltas_geometry",
                            &AgribotControlRosWrapper::backDeltasGeometryServiceCallback, this);
   service_engine_start_ =
      nh_->advertiseService("/agribot/engine_start", &AgribotControlRosWrapper::engineStartService, this);
   service_engine_stop_ =
      nh_->advertiseService("/agribot/engine_stop", &AgribotControlRosWrapper::engineStopService, this);
  callback_function_object_ = boost::bind(&AgribotControlRosWrapper::callbackDynamicReconfigure, this, _1, _2);
  parameter_server_.setCallback(callback_function_object_);
}

void AgribotControlRosWrapper::execute()
{
  // set ros
  ros::Rate loop_rate(10);
  ros::AsyncSpinner spinner(2);  // Use 2 threads
  spinner.start();
  uint8_t encoder_init_timeout_counter = 0;
  while (ros::ok())
  {
    ttcontrol_communication_handler_->receiveCanFrames();
    ttcontrol_communication_handler_->parseCanFrames();
    if (ttcontrol_communication_handler_->checkReceivedCanBusIdBit(ReceivedFramesRegisterMask::velocities))
    {
      const CartsStatus& cart_status = ttcontrol_communication_handler_->getCartsStatus();
      agribot_caterpillar_msg_.header.stamp = ros::Time::now();
      agribot_caterpillar_msg_.header.frame_id = "hal_caterpillar_feedback";
      agribot_caterpillar_msg_.front_linear = cart_status.getFrontLinearVelocity();
      agribot_caterpillar_msg_.front_angular = cart_status.getFrontAngularVelocity();
      agribot_caterpillar_msg_.back_linear = cart_status.getBackLinearVelocity();
      agribot_caterpillar_msg_.back_angular = cart_status.getBackAngularVelocity();
      carts_status_publisher_.publish(agribot_caterpillar_msg_);
      ttcontrol_communication_handler_->clearReceivedCanBusIdBit(ReceivedFramesRegisterMask::velocities);
    }
    if (ttcontrol_communication_handler_->checkReceivedCanBusIdBit(ReceivedFramesRegisterMask::front_angle))
    {
      front_angle_msg_.data = ttcontrol_communication_handler_->getFrontCartAngle();
      front_cart_angle_publisher_.publish(front_angle_msg_);
      ttcontrol_communication_handler_->clearReceivedCanBusIdBit(ReceivedFramesRegisterMask::front_angle);
    }
    if (ttcontrol_communication_handler_->checkReceivedCanBusIdBit(ReceivedFramesRegisterMask::back_angle))
    {
      back_angle_msg_.data = ttcontrol_communication_handler_->getBackCartAngle();
      back_cart_angle_publisher_.publish(back_angle_msg_);
      ttcontrol_communication_handler_->clearReceivedCanBusIdBit(ReceivedFramesRegisterMask::back_angle);
    }
    if (ttcontrol_communication_handler_->checkReceivedCanBusIdBit(ReceivedFramesRegisterMask::automatic_state_request))
    {
      // TODO(Michal/Adam): Implement some diagnostics
      ROS_INFO("[AgribotControlNode] Received automatic mode request");
      usleep(WAIT_FOR_TTCONTROL);
      ttcontrol_communication_handler_->confirmAdvantechReadiness();
      ttcontrol_communication_handler_->clearReceivedCanBusIdBit(ReceivedFramesRegisterMask::automatic_state_request);
    }
    if (ttcontrol_communication_handler_->checkReceivedCanBusIdBit(ReceivedFramesRegisterMask::track_encoder_sum))
    {
      CartsStatus cart_status = ttcontrol_communication_handler_->getCartsStatus();
      encoder_sum_msg_.header.stamp = ros::Time::now();
      encoder_sum_msg_.header.frame_id = "hal_tracks_encoders_sum_feedback";
      encoder_sum_msg_.front_left = cart_status.getTrackEncoderSum(EncoderTrackId::front_left);
      encoder_sum_msg_.front_right = cart_status.getTrackEncoderSum(EncoderTrackId::front_right);
      encoder_sum_msg_.back_left = cart_status.getTrackEncoderSum(EncoderTrackId::back_left);
      encoder_sum_msg_.back_right = cart_status.getTrackEncoderSum(EncoderTrackId::back_right);
      track_encoder_sum_publisher_.publish(encoder_sum_msg_);
      ttcontrol_communication_handler_->clearReceivedCanBusIdBit(ReceivedFramesRegisterMask::track_encoder_sum);
    }
    if (ttcontrol_communication_handler_->checkReceivedCanBusIdBit(
            ReceivedFramesRegisterMask::automatic_state_request_error))
    {
      ROS_ERROR("[AgribotControlNode] Automatic state request error: %d",
                ttcontrol_communication_handler_->getErrorCode());
      ttcontrol_communication_handler_->clearReceivedCanBusIdBit(
          ReceivedFramesRegisterMask::automatic_state_request_error);
    }
    // when connected, read and send encoders value initialization till the confirmation
    if (!ttcontrol_communication_handler_->getEncodersConfirmedStatus() &&
        ttcontrol_communication_handler_->getTtcontrolConnectionStatus())
    {
      if (!readEncodersDataFile())
      {
        throw std::invalid_argument("Failed to read carts angle data file content ! Robots carts encoders values are "
                                    "not "
                                    "inited! \nAttempted path: " +
                                    path_to_carts_encoders_file_);
      }
      ttcontrol_communication_handler_->sendCartsEncodersInitialization();
    }
    // check if the carts has been initialized and write the new received values to file
    if (ttcontrol_communication_handler_->getEncodersConfirmedStatus())
    {
      // check if both angle data reached us
      encoder_init_timeout_counter = 0;
      if (!ttcontrol_communication_handler_->writeEncodersDataFile())
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

TplControl AgribotControlRosWrapper::tpl_control_msg_to_tpl_control_struct(const agribot_msgs::AgribotTool& msg)
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

TtcontrolOverrideParameters
AgribotControlRosWrapper::parseConfigToOverrideParameters(const hal_agribot_control::TTControlParametersConfig& config)
{
  TtcontrolOverrideParameters override_params_data;
  override_params_data.cart_rotation_limit = config.cart_rotation_limit;
  override_params_data.override_velocity_linear = config.override_velocity_linear;
  override_params_data.override_velocity_angular = config.override_velocity_angular;
  override_params_data.override_valves_limits = config.override_valves_limits;
  override_params_data.deltas_geometry_valves_push_limit_front = config.deltas_geometry_valves_push_limit_front;
  override_params_data.deltas_geometry_valves_push_limit_back = config.deltas_geometry_valves_push_limit_back;
  override_params_data.deltas_geometry_valves_pull_front_back = config.deltas_geometry_valves_pull_front_back;
  return override_params_data;
}

void AgribotControlRosWrapper::callbackDynamicReconfigure(const hal_agribot_control::TTControlParametersConfig& config,
                                                          uint32_t level)
{
  // maximum number of queed msg to read in TTControl for a given CAN_ID is 10, so we need to send the msgs
  // in turns
  if (level == TTControlParameterFlag::every_flag_set)
  {
    sendAllParameterGroups(config);
  }
  else
  {
    for (uint64_t bit_flag_iterator = TTControlParameterFlag::lowest_flag_value;
         bit_flag_iterator < TTControlParameterFlag::highest_flag_value; bit_flag_iterator = bit_flag_iterator << 1)
    {
      if (level & bit_flag_iterator)
      {
        sendParameterGroup(bit_flag_iterator, config);
      }
    }
  }
}

void AgribotControlRosWrapper::sendParameterGroup(const uint32_t parameter_group,
                                                  const hal_agribot_control::TTControlParametersConfig& config)
{
  switch (parameter_group)
  {
    case TTControlParameterFlag::signal_params:
      ttcontrol_communication_handler_->sendSignalsParameters(config.ditter_period_int, config.ditter_amplitude_int,
                                                              config.encoder_period_int);
      break;
    case TTControlParameterFlag::front_pid_t_params:
      ttcontrol_communication_handler_->sendCartsPidParameters(config.front_kp_t_double, config.front_ki_t_double,
                                                               config.front_kd_t_double, config.front_acc_t_double,
                                                               TTControlParameterType::front_pid_t);
      break;
    case TTControlParameterFlag::front_pid_r_params:
      ttcontrol_communication_handler_->sendCartsPidParameters(config.front_kp_r_double, config.front_ki_r_double,
                                                               config.front_kd_r_double, config.front_acc_r_double,
                                                               TTControlParameterType::front_pid_r);
      break;
    case TTControlParameterFlag::back_pid_t_params:
      ttcontrol_communication_handler_->sendCartsPidParameters(config.back_kp_t_double, config.back_ki_t_double,
                                                               config.back_kd_t_double, config.back_acc_t_double,
                                                               TTControlParameterType::back_pid_t);
      break;
    case TTControlParameterFlag::back_pid_r_params:
      ttcontrol_communication_handler_->sendCartsPidParameters(config.back_kp_r_double, config.back_ki_r_double,
                                                               config.back_kd_r_double, config.back_acc_r_double,
                                                               TTControlParameterType::back_pid_r);
      break;
    case TTControlParameterFlag::front_left_caterpillar_pid_params:
      ttcontrol_communication_handler_->sendCaterpillarPidParameters(
          config.front_left_caterpillar_kp, config.front_left_caterpillar_ki, config.front_left_caterpillar_kd,
          TTControlParameterType::front_left_caterpillar_pid);
      break;
    case TTControlParameterFlag::front_left_caterpillar_signals_base:
      ttcontrol_communication_handler_->sendCaterpillarBaseSignal(
          config.front_left_caterpillar_signal_base_forward, config.front_left_caterpillar_signal_base_backward,
          TTControlParameterType::front_left_caterpillar_signal_base);
      break;
    case TTControlParameterFlag::front_right_caterpillar_pid_params:
      ttcontrol_communication_handler_->sendCaterpillarPidParameters(
          config.front_right_caterpillar_kp, config.front_right_caterpillar_ki, config.front_right_caterpillar_kd,
          TTControlParameterType::front_right_caterpillar_pid);
      break;
    case TTControlParameterFlag::front_right_caterpillar_signals_base:
      ttcontrol_communication_handler_->sendCaterpillarBaseSignal(
          config.front_right_caterpillar_signal_base_forward, config.front_right_caterpillar_signal_base_backward,
          TTControlParameterType::front_right_caterpillar_signal_base);
      break;
    case TTControlParameterFlag::back_left_caterpillar_pid_params:
      ttcontrol_communication_handler_->sendCaterpillarPidParameters(
          config.back_left_caterpillar_kp, config.back_left_caterpillar_ki, config.back_left_caterpillar_kd,
          TTControlParameterType::back_left_caterpillar_pid);
      break;
    case TTControlParameterFlag::back_left_caterpillar_signals_base:
      ttcontrol_communication_handler_->sendCaterpillarBaseSignal(
          config.back_left_caterpillar_signal_base_forward, config.back_left_caterpillar_signal_base_backward,
          TTControlParameterType::back_left_caterpillar_signal_base);
      break;
    case TTControlParameterFlag::back_right_caterpillar_pid_params:
      ttcontrol_communication_handler_->sendCaterpillarPidParameters(
          config.back_right_caterpillar_kp, config.back_right_caterpillar_ki, config.back_right_caterpillar_kd,
          TTControlParameterType::back_right_caterpillar_pid);
      break;
    case TTControlParameterFlag::back_right_caterpillar_signals_base:
      ttcontrol_communication_handler_->sendCaterpillarBaseSignal(
          config.back_right_caterpillar_signal_base_forward, config.back_right_caterpillar_signal_base_backward,
          TTControlParameterType::back_right_caterpillar_signal_base);
      break;
    case TTControlParameterFlag::override_speed_and_valve_limits_params:
      ttcontrol_communication_handler_->sendOverrideLimits(parseConfigToOverrideParameters(config));
      break;
    case TTControlParameterFlag::stabilization_params:
      ttcontrol_communication_handler_->sendStabilizationParameters(config.stab_pid_kp, config.stab_pid_kd,
                                                                    config.stab_pid_ki, config.stab_pid_max_signal);
      break;
    case TTControlParameterFlag::oil_and_engine_params:
      ttcontrol_communication_handler_->sendOilAndEngineParams(config.max_oil_temperature, config.min_oil_temperature,
                                                               config.histeresis_oil_temperature, config.engine_rpm_max,
                                                               config.engine_rpm_min);
      break;
    default:
      break;
  }
}

void AgribotControlRosWrapper::sendAllParameterGroups(const hal_agribot_control::TTControlParametersConfig& config)
{
  ttcontrol_communication_handler_->sendSignalsParameters(config.ditter_period_int, config.ditter_amplitude_int,
                                                          config.encoder_period_int);
  ttcontrol_communication_handler_->sendCartsPidParameters(config.front_kp_t_double, config.front_ki_t_double,
                                                           config.front_kd_t_double, config.front_acc_t_double,
                                                           TTControlParameterType::front_pid_t);
  ttcontrol_communication_handler_->sendCartsPidParameters(config.front_kp_r_double, config.front_ki_r_double,
                                                           config.front_kd_r_double, config.front_acc_r_double,
                                                           TTControlParameterType::front_pid_r);
  ttcontrol_communication_handler_->sendCartsPidParameters(config.back_kp_t_double, config.back_ki_t_double,
                                                           config.back_kd_t_double, config.back_acc_t_double,
                                                           TTControlParameterType::back_pid_t);
  ttcontrol_communication_handler_->sendCartsPidParameters(config.back_kp_r_double, config.back_ki_r_double,
                                                           config.back_kd_r_double, config.back_acc_r_double,
                                                           TTControlParameterType::back_pid_r);
  ttcontrol_communication_handler_->sendCaterpillarPidParameters(
      config.front_left_caterpillar_kp, config.front_left_caterpillar_ki, config.front_left_caterpillar_kd,
      TTControlParameterType::front_left_caterpillar_pid);

  ros::Duration(0.5).sleep();  // sleep for half a second, to give TTControl time to read and save the parameters
  ttcontrol_communication_handler_->sendCaterpillarBaseSignal(
      config.front_left_caterpillar_signal_base_forward, config.front_left_caterpillar_signal_base_backward,
      TTControlParameterType::front_left_caterpillar_signal_base);
  ttcontrol_communication_handler_->sendCaterpillarPidParameters(
      config.front_right_caterpillar_kp, config.front_right_caterpillar_ki, config.front_right_caterpillar_kd,
      TTControlParameterType::front_right_caterpillar_pid);
  ttcontrol_communication_handler_->sendCaterpillarBaseSignal(
      config.front_right_caterpillar_signal_base_forward, config.front_right_caterpillar_signal_base_backward,
      TTControlParameterType::front_right_caterpillar_signal_base);
  ttcontrol_communication_handler_->sendCaterpillarPidParameters(
      config.back_left_caterpillar_kp, config.back_left_caterpillar_ki, config.back_left_caterpillar_kd,
      TTControlParameterType::back_left_caterpillar_pid);
  ttcontrol_communication_handler_->sendCaterpillarBaseSignal(
      config.back_left_caterpillar_signal_base_forward, config.back_left_caterpillar_signal_base_backward,
      TTControlParameterType::back_left_caterpillar_signal_base);
  ttcontrol_communication_handler_->sendCaterpillarPidParameters(
      config.back_right_caterpillar_kp, config.back_right_caterpillar_ki, config.back_right_caterpillar_kd,
      TTControlParameterType::back_right_caterpillar_pid);
  ttcontrol_communication_handler_->sendCaterpillarBaseSignal(
      config.back_right_caterpillar_signal_base_forward, config.back_right_caterpillar_signal_base_backward,
      TTControlParameterType::back_right_caterpillar_signal_base);

  ros::Duration(0.5).sleep();  // sleep for half a second, to give TTControl time to read and save the parameters
  ttcontrol_communication_handler_->sendOverrideLimits(std::move(parseConfigToOverrideParameters(config)));
  ttcontrol_communication_handler_->sendStabilizationParameters(config.stab_pid_kp, config.stab_pid_kd,
                                                                config.stab_pid_ki, config.stab_pid_max_signal);
  ttcontrol_communication_handler_->sendOilAndEngineParams(config.max_oil_temperature, config.min_oil_temperature,
                                                           config.histeresis_oil_temperature, config.engine_rpm_max,
                                                           config.engine_rpm_min);
}

void AgribotControlRosWrapper::caterpillarsControlCallback(const agribot_msgs::AgribotCaterpillar& msg)
{
  ttcontrol_communication_handler_->sendVelocities(msg.front_linear, msg.front_angular, msg.back_linear,
                                                   msg.back_angular);
}

void AgribotControlRosWrapper::frontTplControlCallback(const agribot_msgs::AgribotTool& msg)
{
  ttcontrol_communication_handler_->sendTplValveControlViaCan(SelectedTpl::front,
                                                              tpl_control_msg_to_tpl_control_struct(msg));
}

void AgribotControlRosWrapper::backTplControlCallback(const agribot_msgs::AgribotTool& msg)
{
  ttcontrol_communication_handler_->sendTplValveControlViaCan(SelectedTpl::back,
                                                              tpl_control_msg_to_tpl_control_struct(msg));
}

bool AgribotControlRosWrapper::sendDeltasGeometryCommand(const RobotsCarts selected_cart,
                                                         const agribot_msgs::ChangeDeltasGeometry::Request& req)
{
  if (req.cart_delta_command == agribot_msgs::ChangeDeltasGeometry::Request::ELEVATED_CATERPILLAR_GEOMETRY ||
      req.cart_delta_command == agribot_msgs::ChangeDeltasGeometry::Request::NORMAL_CATERPILLAR_GEOMETRY)
  {
    if (selected_cart == RobotsCarts::front_cart)
    {
      ttcontrol_communication_handler_->sendDeltasGeometryCommand(
          req.cart_delta_command, agribot_msgs::ChangeDeltasGeometry::Request::UNKNOWN_CATERPILLAR_GEOMETRY);
      return true;
    }
    else
    {
      ttcontrol_communication_handler_->sendDeltasGeometryCommand(
          agribot_msgs::ChangeDeltasGeometry::Request::UNKNOWN_CATERPILLAR_GEOMETRY, req.cart_delta_command);
      return true;
    }
  }
  else
  {
    return false;
  }
}

void AgribotControlRosWrapper::waitTillDeltasGeometryIsSet(const RobotsCarts selected_cart,
                                                           const agribot_msgs::ChangeDeltasGeometry::Request& req)
{
  const CartsStatus& cartsStatus = ttcontrol_communication_handler_->getCartsStatus();
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

void AgribotControlRosWrapper::setDeltasGeometryResult(
    agribot_msgs::ChangeDeltasGeometry::Response& res,  // NOLINT(runtime/references)
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

bool AgribotControlRosWrapper::deltasChangeGeometryServiceExecution(
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

bool AgribotControlRosWrapper::frontDeltasGeometryServiceCallback(
    agribot_msgs::ChangeDeltasGeometry::Request& req,   // NOLINT(runtime/references)
    agribot_msgs::ChangeDeltasGeometry::Response& res)  // NOLINT(runtime/references)
{
  return deltasChangeGeometryServiceExecution(req, res, RobotsCarts::front_cart);
}

bool AgribotControlRosWrapper::backDeltasGeometryServiceCallback(
    agribot_msgs::ChangeDeltasGeometry::Request& req,   // NOLINT(runtime/references)
    agribot_msgs::ChangeDeltasGeometry::Response& res)  // NOLINT(runtime/references)
{
  return deltasChangeGeometryServiceExecution(req, res, RobotsCarts::back_cart);
}

bool AgribotControlRosWrapper::engineStartService(std_srvs::Trigger::Request& req,   // NOLINT(runtime/references)
                                                  std_srvs::Trigger::Response& res)  // NOLINT(runtime/references)
{
  static_cast<void>(req);
  ttcontrol_communication_handler_->sendEngineCommand(TTControlEngineCommands::start_engine);
  // wait till the engine is started
  uint8_t timeout_counter = 0;
  while (!ttcontrol_communication_handler_->getEngineStatus())
  {
    ros::Duration(0.1).sleep();
    timeout_counter++;
    if (timeout_counter > ENGINE_SERVICE_TIMEOUT_LIMIT)
    {
      // send the msg to stop engine to prevent its sudden start when resolving problems causing the failure
      ttcontrol_communication_handler_->sendEngineCommand(TTControlEngineCommands::stop_engine);
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

bool AgribotControlRosWrapper::engineStopService(std_srvs::Trigger::Request& req,   // NOLINT(runtime/references)
                                                 std_srvs::Trigger::Response& res)  // NOLINT(runtime/references)
{
  static_cast<void>(req);
  ttcontrol_communication_handler_->sendEngineCommand(TTControlEngineCommands::stop_engine);
  // wait till the engine is stopped
  uint8_t timeout_counter = 0;
  while (ttcontrol_communication_handler_->getEngineStatus())
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

void AgribotControlRosWrapper::ttcontrolConnectionState(const std_msgs::String& msg)
{
  if (msg.data == TTCONTROL_CONNECTION_MSG)
  {
    ttcontrol_communication_handler_->setTtcontrolConnectionStatus(TTCONTROL_CONNECTED);
  }
  else
  {
    ttcontrol_communication_handler_->setTtcontrolConnectionStatus(TTCONTROL_DISCONNECTED);
  }
}

bool AgribotControlRosWrapper::readEncodersDataFile()
{
  if (!ttcontrol_communication_handler_->readCartsEncodersDataFile(READ_REGULAR_CARTS_ENCODERS_FILE))
  {
    return ttcontrol_communication_handler_->readCartsEncodersDataFile(READ_BACKUP_CARTS_ENCODERS_FILE);
  }
  return true;
}
