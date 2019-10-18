#############################################################################
## Set minimum required version of cmake, project name and compile options ##
#############################################################################
cmake_minimum_required(VERSION 2.8.3)
project(ttcontrol_communication_protocol)

#############################################################################
## Find catkin packages and libraries for catkin and system dependencies   ##
#############################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  agribot_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ttcontrol_communication_protocol
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

#############################################################################
## Build                                                                   ##
#############################################################################

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

link_directories("${PROJECT_SOURCE_DIR}")
add_executable(heartbeat_node src/Can.cpp src/TTCommHB.cpp src/CartsStatus.cpp src/heartbeat_node.cpp)
add_executable(agribot_control_node src/Can.cpp src/TTCommControl.cpp src/CartsStatus.cpp src/agribot_control_node.cpp)

target_link_libraries(heartbeat_node ${catkin_LIBRARIES})
target_link_libraries(agribot_control_node ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############