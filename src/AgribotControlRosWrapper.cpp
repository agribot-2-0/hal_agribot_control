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
  caterpillar_sub_ =
      nh_->subscribe("/agribot/control/caterpillar", 1, &AgribotControlRosWrapper::caterpillarsControlCallback, this);
  tpl_front_control_sub_ =
      nh_->subscribe("/agribot/control/tool/front/valves", 1, &AgribotControlRosWrapper::frontTplControlCallback, this);
  tpl_back_control_sub_ =
      nh_->subscribe("/agribot/control/tool/back/valves", 1, &AgribotControlRosWrapper::backTplControlCallback, this);
}

void AgribotControlRosWrapper::registerPublishers()
{
  carts_status_publisher_ = nh_->advertise<agribot_msgs::AgribotCaterpillar>("/agribot/encoder/caterpillar", 1);
  track_encoder_sum_publisher_ = nh_->advertise<agribot_msgs::AgribotCaterpillarWheelsRotation>(
      "/agribot/encoder/caterpillar/rotations/impulses", 1);
  front_cart_angle_publisher_ = nh_->advertise<std_msgs::Float32>("/agribot/encoder/cart/front", 1);
  back_cart_angle_publisher_ = nh_->advertise<std_msgs::Float32>("/agribot/encoder/cart/back", 1);
  ttcontrol_state_publisher_ = nh_->advertise<agribot_msgs::AgribotTtcontrolState>("/agribot/ttcontrol_state", 1);
}

void AgribotControlRosWrapper::registerServices()
{
  change_delta_front_service_ =
      nh_->advertiseService("/agribot/cart/caterpillar/front/change_deltas_geometry",
                            &AgribotControlRosWrapper::frontDeltasGeometryServiceCallback, this);
  change_delta_back_service_ =
      nh_->advertiseService("/agribot/cart/caterpillar/back/change_deltas_geometry",
                            &AgribotControlRosWrapper::backDeltasGeometryServiceCallback, this);
  engine_start_service_ =
      nh_->advertiseService("/agribot/engine_start", &AgribotControlRosWrapper::engineStartService, this);
  engine_stop_service_ =
      nh_->advertiseService("/agribot/engine_stop", &AgribotControlRosWrapper::engineStopService, this);
  set_autonomous_mode_service_ =
      nh_->advertiseService("/agribot/set_autonomous_mode", &AgribotControlRosWrapper::setAutonomousModeService, this);
  set_safe_state_service_ =
      nh_->advertiseService("/agribot/set_safe_state", &AgribotControlRosWrapper::setSafeStateService, this);
  parameter_server_handler_ =
      std::make_unique<dynamic_reconfigure::Server<hal_agribot_control::TTControlParametersConfig>>();
  callback_function_object_ = boost::bind(&AgribotControlRosWrapper::callbackDynamicReconfigure, this, _1, _2);
  parameter_server_handler_->setCallback(callback_function_object_);
}

AgribotControlRosWrapper::AgribotControlRosWrapper(int argc, char** argv, const char* node_name)
{
  // init ros
  ros::init(argc, argv, node_name);
  nh_ = std::make_unique<ros::NodeHandle>();
  // create TTControl communication handler
  ttcontrol_communication_handler_ = std::make_unique<TTCommControl>();
  // can initialization
  std::string selected_can;
  selected_can = ros_common_functions::getRosParam<std::string>("/hal_agribot_control/can_bus_name");
  ttcontrol_communication_handler_->initializeCanHandler(selected_can);
  // register to ros topics and services
  registerSubscribers();
  registerPublishers();
  registerServices();
  // encoder initialization reading
  path_to_carts_encoders_file_ = ros_common_functions::getRosParam<std::string>("path_to_encoder_file");
  // nh_->getParam("path_to_encoder_file", path_to_carts_encoders_file_);
  ttcontrol_communication_handler_->setPathToCartsEncodersDataFile(path_to_carts_encoders_file_);
  if (!readEncodersDataFile())
  {
    throw std::invalid_argument("Failed to read carts angle data file content ! Robots carts encoders values are not "
                                "inited! \nAttempted path: " +
                                path_to_carts_encoders_file_);
  }
  ttcontrol_communication_handler_->sendCartsEncodersInitialization();
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
    if (ttcontrol_communication_handler_->checkReceivedCanBusIdBit(ReceivedFramesRegisterMask::Velocities))
    {
      const CartsStatus& cart_status = ttcontrol_communication_handler_->getCartsStatus();
      agribot_caterpillar_msg_.header.stamp = ros::Time::now();
      agribot_caterpillar_msg_.header.frame_id = "hal_caterpillar_feedback";
      agribot_caterpillar_msg_.front_linear = cart_status.getFrontLinearVelocity();
      agribot_caterpillar_msg_.front_angular = cart_status.getFrontAngularVelocity();
      agribot_caterpillar_msg_.back_linear = cart_status.getBackLinearVelocity();
      agribot_caterpillar_msg_.back_angular = cart_status.getBackAngularVelocity();
      carts_status_publisher_.publish(agribot_caterpillar_msg_);
      ttcontrol_communication_handler_->clearReceivedCanBusIdBit(ReceivedFramesRegisterMask::Velocities);
    }
    if (ttcontrol_communication_handler_->checkReceivedCanBusIdBit(ReceivedFramesRegisterMask::FrontAngle))
    {
      front_angle_msg_.data = ttcontrol_communication_handler_->getFrontCartAngle();
      front_cart_angle_publisher_.publish(front_angle_msg_);
      ttcontrol_communication_handler_->clearReceivedCanBusIdBit(ReceivedFramesRegisterMask::FrontAngle);
    }
    if (ttcontrol_communication_handler_->checkReceivedCanBusIdBit(ReceivedFramesRegisterMask::BackAngle))
    {
      back_angle_msg_.data = ttcontrol_communication_handler_->getBackCartAngle();
      back_cart_angle_publisher_.publish(back_angle_msg_);
      ttcontrol_communication_handler_->clearReceivedCanBusIdBit(ReceivedFramesRegisterMask::BackAngle);
    }
    if (ttcontrol_communication_handler_->checkReceivedCanBusIdBit(ReceivedFramesRegisterMask::TtcontrolStates))
    {
      ttcontrol_states_msg_.automatic_state =
          static_cast<uint8_t>(ttcontrol_communication_handler_->getTtcontrolCurrentState().automatic_state);
      ttcontrol_states_msg_.manual_state =
          static_cast<uint8_t>(ttcontrol_communication_handler_->getTtcontrolCurrentState().manual_state);
      if (ttcontrol_communication_handler_->getEncodersConfirmedStatus())
      {
        ttcontrol_states_msg_.encoders_inited = agribot_msgs::AgribotTtcontrolState::ENCODERS_INITED;
      }
      else
      {
        ttcontrol_states_msg_.encoders_inited = agribot_msgs::AgribotTtcontrolState::ENCODERS_NOT_INITED;
      }
      ttcontrol_state_publisher_.publish(ttcontrol_states_msg_);
      ttcontrol_communication_handler_->clearReceivedCanBusIdBit(ReceivedFramesRegisterMask::TtcontrolStates);
    }
    if (ttcontrol_communication_handler_->checkReceivedCanBusIdBit(ReceivedFramesRegisterMask::AutomaticStateRequest))
    {
      // TODO(Michal/Adam): Implement some diagnostics
      ROS_INFO("[HalAgribotControl] Received automatic mode request");
      usleep(TimeoutsLimits::WaitForTtcontrol);
      ttcontrol_communication_handler_->clearReceivedCanBusIdBit(ReceivedFramesRegisterMask::AutomaticStateRequest);
    }
    if (ttcontrol_communication_handler_->checkReceivedCanBusIdBit(ReceivedFramesRegisterMask::TrackEncoderSum))
    {
      CartsStatus cart_status = ttcontrol_communication_handler_->getCartsStatus();
      encoder_sum_msg_.header.stamp = ros::Time::now();
      encoder_sum_msg_.header.frame_id = "hal_tracks_encoders_sum_feedback";
      encoder_sum_msg_.front_left = cart_status.getTrackEncoderSum(EncoderTrackId::FrontLeft);
      encoder_sum_msg_.front_right = cart_status.getTrackEncoderSum(EncoderTrackId::FrontRight);
      encoder_sum_msg_.back_left = cart_status.getTrackEncoderSum(EncoderTrackId::BackLeft);
      encoder_sum_msg_.back_right = cart_status.getTrackEncoderSum(EncoderTrackId::BackRight);
      track_encoder_sum_publisher_.publish(encoder_sum_msg_);
      ttcontrol_communication_handler_->clearReceivedCanBusIdBit(ReceivedFramesRegisterMask::TrackEncoderSum);
    }
    if (ttcontrol_communication_handler_->checkReceivedCanBusIdBit(ReceivedFramesRegisterMask::TtcontrolStates))
    {
      ttcontrol_communication_handler_->getTtcontrolCurrentState().automatic_state;
      ttcontrol_communication_handler_->getTtcontrolCurrentState().manual_state;
      ttcontrol_communication_handler_->clearReceivedCanBusIdBit(ReceivedFramesRegisterMask::TtcontrolStates);
    }
    if (ttcontrol_communication_handler_->checkReceivedCanBusIdBit(
            ReceivedFramesRegisterMask::AutomaticStateRequestError))
    {
      ROS_ERROR("[HalAgribotControl] Automatic state request error: %d",
                ttcontrol_communication_handler_->getErrorCode());
      ttcontrol_communication_handler_->clearReceivedCanBusIdBit(
          ReceivedFramesRegisterMask::AutomaticStateRequestError);
    }
    // when connected, read and send encoders value initialization till the confirmation
    if (!ttcontrol_communication_handler_->getEncodersConfirmedStatus() &&
        ttcontrol_communication_handler_->getTtcontrolConnectionStatus())
    {
      if (!readEncodersDataFile())
      {
        throw std::invalid_argument(  //
            "Failed to read carts angle data file content ! Robots carts encoders values are not inited!"
            "\nAttempted path: " +
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
        ROS_ERROR("[HalAgribotControl] Failed to write carts encoders data to file!");
      }
    }
    else
    {
      if (encoder_init_timeout_counter >= TimeoutsLimits::CartsEncodersInitTimeoutLimit)
      {
        ROS_ERROR("[HalAgribotControl] Carts encoders not inited!");
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

TplControl AgribotControlRosWrapper::agribotToolMsgToTplControlStruct(const agribot_msgs::AgribotTool& msg)
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
  if (level == TTControlParameterFlag::EveryFlagSet)
  {
    sendAllParameterGroups(config);
  }
  else
  {
    for (uint64_t bit_flag_iterator = TTControlParameterFlag::LowestFlagValue;
         bit_flag_iterator < TTControlParameterFlag::HighestFlagValue; bit_flag_iterator = bit_flag_iterator << 1)
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
    case TTControlParameterFlag::SignalParams:
      ttcontrol_communication_handler_->sendSignalsParameters(config.ditter_period_int, config.ditter_amplitude_int,
                                                              config.encoder_period_int);
      break;
    case TTControlParameterFlag::FrontPidTranslationParams:
      ttcontrol_communication_handler_->sendCartsPidParameters(config.front_kp_t_double, config.front_ki_t_double,
                                                               config.front_kd_t_double, config.front_acc_t_double,
                                                               TTControlParameterType::FrontPidT);
      break;
    case TTControlParameterFlag::FrontPidRotationParams:
      ttcontrol_communication_handler_->sendCartsPidParameters(config.front_kp_r_double, config.front_ki_r_double,
                                                               config.front_kd_r_double, config.front_acc_r_double,
                                                               TTControlParameterType::FrontPidR);
      break;
    case TTControlParameterFlag::BackPidTranslationParams:
      ttcontrol_communication_handler_->sendCartsPidParameters(config.back_kp_t_double, config.back_ki_t_double,
                                                               config.back_kd_t_double, config.back_acc_t_double,
                                                               TTControlParameterType::BackPidT);
      break;
    case TTControlParameterFlag::BackPidRotationParams:
      ttcontrol_communication_handler_->sendCartsPidParameters(config.back_kp_r_double, config.back_ki_r_double,
                                                               config.back_kd_r_double, config.back_acc_r_double,
                                                               TTControlParameterType::BackPidR);
      break;
    case TTControlParameterFlag::FrontLeftCaterpillarPidParams:
      ttcontrol_communication_handler_->sendCaterpillarPidParameters(
          config.front_left_caterpillar_kp, config.front_left_caterpillar_ki, config.front_left_caterpillar_kd,
          TTControlParameterType::FrontLeftCaterpillarPid);
      break;
    case TTControlParameterFlag::FrontLeftCaterpillarSignalsBase:
      ttcontrol_communication_handler_->sendCaterpillarBaseSignal(
          config.front_left_caterpillar_signal_base_forward, config.front_left_caterpillar_signal_base_backward,
          TTControlParameterType::FrontLeftCaterpillarSignalBase);
      break;
    case TTControlParameterFlag::FrontRightCaterpillarPidParams:
      ttcontrol_communication_handler_->sendCaterpillarPidParameters(
          config.front_right_caterpillar_kp, config.front_right_caterpillar_ki, config.front_right_caterpillar_kd,
          TTControlParameterType::FrontRightCaterpillarPid);
      break;
    case TTControlParameterFlag::FrontRightCaterpillarSignalsBase:
      ttcontrol_communication_handler_->sendCaterpillarBaseSignal(
          config.front_right_caterpillar_signal_base_forward, config.front_right_caterpillar_signal_base_backward,
          TTControlParameterType::FrontRightCaterpillarSignalBase);
      break;
    case TTControlParameterFlag::BackLeftCaterpillarPidParams:
      ttcontrol_communication_handler_->sendCaterpillarPidParameters(
          config.back_left_caterpillar_kp, config.back_left_caterpillar_ki, config.back_left_caterpillar_kd,
          TTControlParameterType::BackLeftCaterpillarPid);
      break;
    case TTControlParameterFlag::BackLeftCaterpillarSignalsBase:
      ttcontrol_communication_handler_->sendCaterpillarBaseSignal(
          config.back_left_caterpillar_signal_base_forward, config.back_left_caterpillar_signal_base_backward,
          TTControlParameterType::BackLeftCaterpillarSignalBase);
      break;
    case TTControlParameterFlag::BackRightCaterpillarPidParams:
      ttcontrol_communication_handler_->sendCaterpillarPidParameters(
          config.back_right_caterpillar_kp, config.back_right_caterpillar_ki, config.back_right_caterpillar_kd,
          TTControlParameterType::BackRightCaterpillarPid);
      break;
    case TTControlParameterFlag::BackRightCaterpillarSignalsBase:
      ttcontrol_communication_handler_->sendCaterpillarBaseSignal(
          config.back_right_caterpillar_signal_base_forward, config.back_right_caterpillar_signal_base_backward,
          TTControlParameterType::BackRightCaterpillarSignalBase);
      break;
    case TTControlParameterFlag::OverrideSpeedAndValveLimitsParams:
      ttcontrol_communication_handler_->sendOverrideLimits(parseConfigToOverrideParameters(config));
      break;
    case TTControlParameterFlag::StabilizationParams:
      ttcontrol_communication_handler_->sendStabilizationParameters(
          config.chassis_stabilization_pid_kp, config.chassis_stabilization_pid_kd, config.chassis_stabilization_pid_ki,
          config.chassis_stabilization_pid_max_signal);
      break;
    case TTControlParameterFlag::OilAndEngineParams:
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
  // maximum number of queued msg to read in TTControl for a given CAN_ID is 10, so we need to send the msgs
  // in turns of arbitratry number of frames lower than 10
  uint8_t arbitrary_group_count = 6;
  uint8_t group_delay_counter = 0;

  for (uint64_t bit_flag_iterator = TTControlParameterFlag::LowestFlagValue;
       bit_flag_iterator < TTControlParameterFlag::HighestFlagValue; bit_flag_iterator = bit_flag_iterator << 1)
  {
    if (TTControlParameterFlag::EveryFlagSet & bit_flag_iterator)
    {
      sendParameterGroup(bit_flag_iterator, config);
    }
    // delay procedure to give ttcontrol time to process
    group_delay_counter++;
    if (group_delay_counter % arbitrary_group_count == 0)
    {
      ros::Duration(0.5).sleep();  // sleep for half a second, to give TTControl time to read and save the parameters
    }
  }
}

void AgribotControlRosWrapper::caterpillarsControlCallback(const agribot_msgs::AgribotCaterpillar& msg)
{
  ttcontrol_communication_handler_->sendVelocities(msg.front_linear, msg.front_angular, msg.back_linear,
                                                   msg.back_angular);
}

void AgribotControlRosWrapper::frontTplControlCallback(const agribot_msgs::AgribotTool& msg)
{
  ttcontrol_communication_handler_->sendTplValveControlViaCan(SelectedTpl::Front,
                                                              agribotToolMsgToTplControlStruct(msg));
}

void AgribotControlRosWrapper::backTplControlCallback(const agribot_msgs::AgribotTool& msg)
{
  ttcontrol_communication_handler_->sendTplValveControlViaCan(SelectedTpl::Back, agribotToolMsgToTplControlStruct(msg));
}

bool AgribotControlRosWrapper::sendDeltasGeometryCommand(const Cart selected_cart,
                                                         const agribot_msgs::ChangeDeltasGeometry::Request& req)
{
  if (req.cart_delta_command == agribot_msgs::ChangeDeltasGeometry::Request::ELEVATED_CATERPILLAR_GEOMETRY ||
      req.cart_delta_command == agribot_msgs::ChangeDeltasGeometry::Request::NORMAL_CATERPILLAR_GEOMETRY)
  {
    if (selected_cart == Cart::Front)
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

bool AgribotControlRosWrapper::waitTillDeltasGeometryIsSet(const Cart selected_cart,
                                                           const agribot_msgs::ChangeDeltasGeometry::Request& req)
{
  const CartsStatus& cartsStatus = ttcontrol_communication_handler_->getCartsStatus();
  uint8_t cart_state = agribot_msgs::ChangeDeltasGeometry::Response::UNKNOWN_CATERPILLAR_GEOMETRY;
  uint16_t request_timeout_limit_counter = 0;
  if (selected_cart == Cart::Front)
  {
    while (cart_state != req.cart_delta_command)
    {
      if (request_timeout_limit_counter == TimeoutsLimits::DeltasExtensionTimeoutLimit)
      {
        ttcontrol_communication_handler_->sendDeltasGeometryCommand(
            agribot_msgs::ChangeDeltasGeometry::Request::UNKNOWN_CATERPILLAR_GEOMETRY,
            agribot_msgs::ChangeDeltasGeometry::Request::UNKNOWN_CATERPILLAR_GEOMETRY);
        return false;
      }
      cart_state = cartsStatus.getFrontCartGeometryState();
      ros::Duration(0.1).sleep();
      request_timeout_limit_counter++;
    }
    return true;
  }
  else
  {
    while (cart_state != req.cart_delta_command)
    {
      if (request_timeout_limit_counter == TimeoutsLimits::DeltasExtensionTimeoutLimit)
      {
        ttcontrol_communication_handler_->sendDeltasGeometryCommand(
            agribot_msgs::ChangeDeltasGeometry::Request::UNKNOWN_CATERPILLAR_GEOMETRY,
            agribot_msgs::ChangeDeltasGeometry::Request::UNKNOWN_CATERPILLAR_GEOMETRY);
        return false;
      }
      cart_state = cartsStatus.getBackCartGeometryState();
      ros::Duration(0.1).sleep();
      request_timeout_limit_counter++;
    }
    return true;
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
    const Cart selected_cart)
{
  if (sendDeltasGeometryCommand(selected_cart, req))
  {
    if (!waitTillDeltasGeometryIsSet(selected_cart, req))
    {
      res.result = res.UNKNOWN_CATERPILLAR_GEOMETRY;
      return true;
    }
    setDeltasGeometryResult(res, req);
    return true;
  }
  else
  {
    res.result = res.UNKNOWN_CATERPILLAR_GEOMETRY;
    return true;
  }
}

bool AgribotControlRosWrapper::frontDeltasGeometryServiceCallback(
    agribot_msgs::ChangeDeltasGeometry::Request& req,   // NOLINT(runtime/references)
    agribot_msgs::ChangeDeltasGeometry::Response& res)  // NOLINT(runtime/references)
{
  return deltasChangeGeometryServiceExecution(req, res, Cart::Front);
}

bool AgribotControlRosWrapper::backDeltasGeometryServiceCallback(
    agribot_msgs::ChangeDeltasGeometry::Request& req,   // NOLINT(runtime/references)
    agribot_msgs::ChangeDeltasGeometry::Response& res)  // NOLINT(runtime/references)
{
  return deltasChangeGeometryServiceExecution(req, res, Cart::Back);
}

bool AgribotControlRosWrapper::engineStartService(std_srvs::Trigger::Request& req,   // NOLINT(runtime/references)
                                                  std_srvs::Trigger::Response& res)  // NOLINT(runtime/references)
{
  static_cast<void>(req);
  ttcontrol_communication_handler_->sendEngineCommand(TTControlEngineCommands::StartEngine);
  // wait till the engine is started
  uint8_t timeout_counter = 0;
  while (!ttcontrol_communication_handler_->getEngineStatus())
  {
    ros::Duration(0.1).sleep();
    timeout_counter++;
    if (timeout_counter > TimeoutsLimits::EngineServiceTimeoutLimit)
    {
      // send the msg to stop engine to prevent its sudden start when resolving problems causing the failure
      ttcontrol_communication_handler_->sendEngineCommand(TTControlEngineCommands::StopEngine);
      res.success = false;
      res.message = "Couldnt start engine, please check: \n- voltage level";
      res.message += "\n- fuel level \n- can interface connection";
      return true;
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
  ttcontrol_communication_handler_->sendEngineCommand(TTControlEngineCommands::StopEngine);
  // wait till the engine is stopped
  uint8_t timeout_counter = 0;
  while (ttcontrol_communication_handler_->getEngineStatus())
  {
    ros::Duration(0.1).sleep();
    timeout_counter++;
    if (timeout_counter > TimeoutsLimits::EngineServiceTimeoutLimit)
    {
      res.success = false;
      res.message = "Couldnt stop engine, please check: \n- can interface connection";
      return true;
    }
  }
  res.success = true;
  res.message = "Engine stopped";
  return true;
}

bool AgribotControlRosWrapper::setAutonomousModeService(std_srvs::Trigger::Request& req,   // NOLINT(runtime/references)
                                                        std_srvs::Trigger::Response& res)  // NOLINT(runtime/references)
{
  static_cast<void>(req);
  if (ttcontrol_communication_handler_->getTtcontrolCurrentState().manual_state == TtcontrolState::SteeringActive)
  {
    res.success = false;
    res.message = "Couldn't start the autonomous steering, because manual steering is still active ";
    return true;
  }
  if (ttcontrol_communication_handler_->getTtcontrolCurrentState().automatic_state == TtcontrolState::SteeringActive)
  {
    res.success = false;
    res.message = "Couldn't start the autonomous steering, because its already active ";
    return true;
  }
  else if (ttcontrol_communication_handler_->getTtcontrolCurrentState().automatic_state ==
               TtcontrolState::Initialization ||
           ttcontrol_communication_handler_->getTtcontrolCurrentState().automatic_state ==
               TtcontrolState::WaitingForHandshake)
  {
    res.success = false;
    res.message = "Couldn't start the autonomous steering, because there is no connection with TTControl";
    return true;
  }
  else if (ttcontrol_communication_handler_->getTtcontrolCurrentState().automatic_state ==
           TtcontrolState::CommunicationActive)
  {
    uint8_t timeout_counter = 0;
    ttcontrol_communication_handler_->requestChangeOfControlState(
        TtcontrolAutomaticModeRequestByteValues::EnterAutomaticSteering);
    while (ttcontrol_communication_handler_->getTtcontrolCurrentState().automatic_state !=
           TtcontrolState::SteeringActive)
    {
      ros::Duration(0.1).sleep();
      timeout_counter++;
      if (timeout_counter > TimeoutsLimits::ChangeTtcontrolStateRequestTimeoutLimit)
      {
        // withdraw the previous request
        ttcontrol_communication_handler_->requestChangeOfControlState(
            TtcontrolAutomaticModeRequestByteValues::EnterSafeStateRequest);
        res.success = false;
        res.message = "Couldn't start the autonomous steerings, request timeout";
        return true;
      }
    }
  }
  res.success = true;
  return true;
}

bool AgribotControlRosWrapper::setSafeStateService(std_srvs::Trigger::Request& req,   // NOLINT(runtime/references)
                                                   std_srvs::Trigger::Response& res)  // NOLINT(runtime/references)
{
  static_cast<void>(req);
  uint8_t timeout_counter = 0;
  // send safe state request till the steering is active
  while (ttcontrol_communication_handler_->getTtcontrolCurrentState().automatic_state == TtcontrolState::SteeringActive)
  {
    ros::Duration(0.1).sleep();
    ttcontrol_communication_handler_->requestChangeOfControlState(
        TtcontrolAutomaticModeRequestByteValues::EnterSafeStateRequest);
    timeout_counter++;
    if (timeout_counter > TimeoutsLimits::ChangeTtcontrolStateRequestTimeoutLimit)
    {
      res.success = false;
      res.message = "Couldn't stop the autonomous steerings, request timeout";
      return true;
    }
  }
  res.success = true;
  return true;
}

bool AgribotControlRosWrapper::readEncodersDataFile()
{
  if (!ttcontrol_communication_handler_->readCartsEncodersDataFile(EncodersFileState::ReadRegularCartsEncodersFile))
  {
    return ttcontrol_communication_handler_->readCartsEncodersDataFile(EncodersFileState::ReadBackupCartsEncodersFile);
  }
  return true;
}
