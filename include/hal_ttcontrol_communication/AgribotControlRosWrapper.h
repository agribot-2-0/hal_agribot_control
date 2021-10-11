/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once
#include <agribot_common/CommonFunctions/ros_common_functions.h>
#include <agribot_msgs/AgribotCaterpillar.h>
#include <agribot_msgs/AgribotCaterpillarWheelsRotation.h>
#include <agribot_msgs/AgribotTool.h>
#include <agribot_msgs/AgribotTtcontrolState.h>
#include <agribot_msgs/ChangeDeltasGeometry.h>
#include <dynamic_reconfigure/server.h>
#include <hal_agribot_control/TTControlParametersConfig.h>
#include <hal_ttcontrol_communication/TTCommControl.h>
#include <hal_ttcontrol_communication/TplControl.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

constexpr std::string_view TTCONTROL_CONNECTION_MSG = "Connected";

enum TimeoutsLimits : uint
{
  WaitForTtcontrol = 100000,
  CartsEncodersInitTimeoutLimit = 10,
  ChangeTtcontrolStateRequestTimeoutLimit = 20,
  EngineServiceTimeoutLimit = 60,
  DeltasExtensionTimeoutLimit = 150
};

enum EncodersFileState : bool
{
  ReadBackupCartsEncodersFile = false,
  ReadRegularCartsEncodersFile = true
};

enum TtcontrolConnectionStatus : bool
{
  Connected = true,
  Disconnected = false
};

enum Cart : std::uint8_t
{
  Front = 1,
  Back = 0
};

enum TTControlParameterFlag : std::uint32_t
{
  LowestFlagValue = 0x01,
  SignalParams = 0x01,
  FrontPidTranslationParams = 0x02,
  FrontPidRotationParams = 0x04,
  BackPidTranslationParams = 0x08,
  BackPidRotationParams = 0x10,
  FrontLeftCaterpillarPidParams = 0x20,
  FrontLeftCaterpillarSignalsBase = 0x40,
  FrontRightCaterpillarPidParams = 0x80,
  FrontRightCaterpillarSignalsBase = 0x100,
  BackLeftCaterpillarPidParams = 0x200,
  BackLeftCaterpillarSignalsBase = 0x400,
  BackRightCaterpillarPidParams = 0x800,
  BackRightCaterpillarSignalsBase = 0x1000,
  OverrideSpeedAndValveLimitsParams = 0x2000,
  StabilizationParams = 0x4000,
  OilAndEngineParams = 0x8000,
  HighestFlagValue = 0x80000000,
  EveryFlagSet = 0xFFFFFFFF
};

class AgribotControlRosWrapper
{
  std::unique_ptr<ros::NodeHandle> nh_;
  std::unique_ptr<TTCommControl> ttcontrol_communication_handler_;
  std_msgs::Float32 front_angle_msg_;
  std_msgs::Float32 back_angle_msg_;
  agribot_msgs::AgribotTtcontrolState ttcontrol_states_msg_;
  agribot_msgs::AgribotCaterpillar agribot_caterpillar_msg_;
  agribot_msgs::AgribotCaterpillarWheelsRotation encoder_time_diff_msg_;
  std::string path_to_carts_encoders_file_;

  ros::Subscriber caterpillar_sub_;
  ros::Subscriber tpl_front_control_sub_;
  ros::Subscriber tpl_back_control_sub_;
  ros::Publisher carts_status_publisher_;
  ros::Publisher track_encoder_sum_publisher_;
  ros::Publisher track_encoder_speed_publisher_;
  ros::Publisher track_encoder_speed_measured_publisher_;
  ros::Publisher track_encoder_speed_desired_publisher_;
  ros::Publisher track_control_publisher_;
  ros::Publisher front_cart_angle_publisher_;
  ros::Publisher back_cart_angle_publisher_;
  ros::Publisher ttcontrol_state_publisher_;
  std::unique_ptr<dynamic_reconfigure::Server<hal_agribot_control::TTControlParametersConfig>>
      parameter_server_handler_;
  dynamic_reconfigure::Server<hal_agribot_control::TTControlParametersConfig>::CallbackType callback_function_object_;
  ros::ServiceServer change_delta_front_service_;
  ros::ServiceServer change_delta_back_service_;
  ros::ServiceServer engine_start_service_;
  ros::ServiceServer engine_stop_service_;
  ros::ServiceServer set_autonomous_mode_service_;
  ros::ServiceServer set_safe_state_service_;

  bool readEncodersDataFile();
  bool engineStopService(std_srvs::Trigger::Request& req,     // NOLINT(runtime/references)
                         std_srvs::Trigger::Response& res);   // NOLINT(runtime/references)
  bool engineStartService(std_srvs::Trigger::Request& req,    // NOLINT(runtime/references)
                          std_srvs::Trigger::Response& res);  // NOLINT(runtime/references)
  bool
  backDeltasGeometryServiceCallback(agribot_msgs::ChangeDeltasGeometry::Request& req,    // NOLINT(runtime/references)
                                    agribot_msgs::ChangeDeltasGeometry::Response& res);  // NOLINT(runtime/references)
  bool
  frontDeltasGeometryServiceCallback(agribot_msgs::ChangeDeltasGeometry::Request& req,    // NOLINT(runtime/references)
                                     agribot_msgs::ChangeDeltasGeometry::Response& res);  // NOLINT(runtime/references)
  bool setAutonomousModeService(std_srvs::Trigger::Request& req,                          // NOLINT(runtime/references)
                                std_srvs::Trigger::Response& res);                        // NOLINT(runtime/references)
  bool setSafeStateService(std_srvs::Trigger::Request& req,                               // NOLINT(runtime/references)
                           std_srvs::Trigger::Response& res);                             // NOLINT(runtime/references)
  bool
  deltasChangeGeometryServiceExecution(const agribot_msgs::ChangeDeltasGeometry::Request& req,
                                       agribot_msgs::ChangeDeltasGeometry::Response& res,  // NOLINT(runtime/references)
                                       const Cart selected_cart);
  void setDeltasGeometryResult(agribot_msgs::ChangeDeltasGeometry::Response& res,  // NOLINT(runtime/references)
                               const agribot_msgs::ChangeDeltasGeometry::Request& req);
  bool waitTillDeltasGeometryIsSet(const Cart selected_cart, const agribot_msgs::ChangeDeltasGeometry::Request& req);
  bool sendDeltasGeometryCommand(const Cart selected_cart, const agribot_msgs::ChangeDeltasGeometry::Request& req);
  void backTplControlCallback(const agribot_msgs::AgribotTool& msg);
  void frontTplControlCallback(const agribot_msgs::AgribotTool& msg);
  void caterpillarsControlCallback(const agribot_msgs::AgribotCaterpillar& msg);
  void callbackDynamicReconfigure(const hal_agribot_control::TTControlParametersConfig& config, std::uint32_t level);
  void sendParameterGroup(const std::uint32_t parameter_group,
                          const hal_agribot_control::TTControlParametersConfig& config);
  void sendAllParameterGroups(const hal_agribot_control::TTControlParametersConfig& config);
  TtcontrolOverrideParameters
  parseConfigToOverrideParameters(const hal_agribot_control::TTControlParametersConfig& config);
  TplControl agribotToolMsgToTplControlStruct(const agribot_msgs::AgribotTool& msg);
  void registerSubscribers();
  void registerPublishers();
  void registerServices();

  double miliseconds_to_seconds(double miliseconds);

public:
  AgribotControlRosWrapper(int argc, char** argv, const char* node_name);
  void execute();
};
