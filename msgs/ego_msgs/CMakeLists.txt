cmake_minimum_required(VERSION 3.0.2)
project(ego_msgs)

add_definitions(-std=c++14)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  cmake_modules
)

set(CMAKE_BUILD_TYPE Release)

##########################
## Declare ROS messages ##
##########################

add_message_files(
  FILES
  UpperBodyState.msg
  LowerBodyState.msg
  DebugLQR.msg
  RPY.msg
  EgoPose2DUnicycle.msg
  EgoTwist2DUnicycle.msg
  EgoQbAdvanceRef.msg
  ArmTorque.msg
  EgoArms.msg
  Wheel_Torque.msg
  AlteregoState.msg
  DetectedObjectInfo.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

##############################
## Declare a C++ executable ##
##############################

