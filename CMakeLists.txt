#############################################################################
## Set minimum required version of cmake, project name and compile options ##
#############################################################################
cmake_minimum_required(VERSION 2.8.3)
project(hal_agribot_control)

#############################################################################
## Find catkin packages and libraries for catkin and system dependencies   ##
#############################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  agribot_msgs
  agribot_common
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
#  LIBRARIES hal_agribot_control
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

#############################################################################
## Build                                                                   ##
#############################################################################

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CMAKE_SOURCE_DIR}/agribot_common/include
)

link_directories("${PROJECT_SOURCE_DIR}")
add_executable(heartbeat_node src/TTCommHB.cpp src/CartsStatus.cpp src/heartbeat_node.cpp)
add_executable(agribot_control_node src/TTCommControl.cpp src/CartsStatus.cpp src/agribot_control_node.cpp)

target_link_libraries(heartbeat_node ${agribot_common_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(agribot_control_node ${agribot_common_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(heartbeat_node ${catkin_EXPORTED_TARGETS})
add_dependencies(agribot_control_node ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

#############
## Testing ##
#############
