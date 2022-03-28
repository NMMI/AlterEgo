cmake_minimum_required(VERSION 2.8.3)
project(invdyn_controller)

add_definitions(-std=c++11)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  kdl_parser
  message_generation
  cmake_modules
  ego_msgs
)

FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)
FIND_PACKAGE (Eigen3 REQUIRED)
find_package(Eigen REQUIRED)

set(CMAKE_BUILD_TYPE Release)

##########################
## Declare ROS messages ##
##########################

add_message_files(
  FILES
  Arm_Des_Pos.msg
  Arm_Des_Vel.msg
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
  INCLUDE_DIRS include
#  LIBRARIES invdyn_controller
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
  DEPENDS Eigen
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

##############################
## Declare a C++ executable ##
##############################

add_executable(invdyn_controller src/InvdynController.cpp src/main_controller.cpp )

add_dependencies(invdyn_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(invdyn_controller
  ${catkin_LIBRARIES} 
  ${RBDL_LIBRARY}
  ${RBDL_URDFReader_LIBRARY}
)

