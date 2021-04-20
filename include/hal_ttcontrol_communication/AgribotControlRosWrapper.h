#pragma once
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

constexpr uint WAIT_FOR_TTCONTROL = 100000;
constexpr uint8_t CARTS_ENCODERS_INIT_TIMEOUT_LIMIT = 10;
constexpr uint8_t ENGINE_SERVICE_TIMEOUT_LIMIT = 60;
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

class AgribotControlRosWrapper
{
  std::unique_ptr<ros::NodeHandle> nh_;
  std::unique_ptr<TTCommControl> ttcontrol_communication_handler_;
  std_msgs::Float32 front_angle_msg_;
  std_msgs::Float32 back_angle_msg_;
  agribot_msgs::AgribotCaterpillar agribot_caterpillar_msg_;
  agribot_msgs::AgribotCaterpillarWheelsRotation encoder_sum_msg_;
  std::string path_to_carts_encoders_file_;

  ros::Subscriber sub_caterpillar_;
  ros::Subscriber sub_connection_state_;
  ros::Subscriber tpl_front_control_sub_;
  ros::Subscriber tpl_back_control_sub_;
  ros::Publisher carts_status_publisher_;
  ros::Publisher track_encoder_sum_publisher_;
  ros::Publisher front_cart_angle_publisher_;
  ros::Publisher back_cart_angle_publisher_;
  dynamic_reconfigure::Server<hal_agribot_control::TTControlParametersConfig> parameter_server_;
  dynamic_reconfigure::Server<hal_agribot_control::TTControlParametersConfig>::CallbackType callback_function_object_;
  ros::ServiceServer service_front_;
  ros::ServiceServer service_back_;
  ros::ServiceServer service_engine_start_;
  ros::ServiceServer service_engine_stop_;

  bool readEncodersDataFile();
  void ttcontrolConnectionState(const std_msgs::String& msg);
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
  bool
  deltasChangeGeometryServiceExecution(const agribot_msgs::ChangeDeltasGeometry::Request& req,
                                       agribot_msgs::ChangeDeltasGeometry::Response& res,  // NOLINT(runtime/references)
                                       const RobotsCarts selected_cart);
  void setDeltasGeometryResult(agribot_msgs::ChangeDeltasGeometry::Response& res,  // NOLINT(runtime/references)
                               const agribot_msgs::ChangeDeltasGeometry::Request& req);
  void waitTillDeltasGeometryIsSet(const RobotsCarts selected_cart,
                                   const agribot_msgs::ChangeDeltasGeometry::Request& req);
  bool sendDeltasGeometryCommand(const RobotsCarts selected_cart,
                                 const agribot_msgs::ChangeDeltasGeometry::Request& req);
  void backTplControlCallback(const agribot_msgs::AgribotTool& msg);
  void frontTplControlCallback(const agribot_msgs::AgribotTool& msg);
  void caterpillarsControlCallback(const agribot_msgs::AgribotCaterpillar& msg);
  void callbackDynamicReconfigure(const hal_agribot_control::TTControlParametersConfig& config, uint32_t level);
  TtcontrolOverrideParameters&&
  parseConfigToOverrideParameters(const hal_agribot_control::TTControlParametersConfig& config);
  TplControl tpl_control_msg_to_tpl_control_struct(const agribot_msgs::AgribotTool& msg);
  void registerSubscribers();
  void registerPublishers();
  void registerServices();

public:
  AgribotControlRosWrapper(int argc, char** argv);
  void execute();
};
