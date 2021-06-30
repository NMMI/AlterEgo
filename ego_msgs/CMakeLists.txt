cmake_minimum_required(VERSION 2.8.3)
project(ego_msgs)

add_definitions(-std=c++11)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  cmake_modules
)

set(CMAKE_BUILD_TYPE Release)

##########################
## Declare ROS messages ##
##########################

add_message_files(
  FILES
  EgoPose2DUnicycle.msg
  EgoTwist2DUnicycle.msg
  EgoQbAdvanceRef.msg
  Arm_Torque.msg
  Wheel_Torque.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
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

