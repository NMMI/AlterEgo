cmake_minimum_required(VERSION 2.8.3)
project(lqr_controller)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  kdl_parser
  cmake_modules
  ego_msgs
)

FIND_PACKAGE (Eigen3 REQUIRED)
find_package(Eigen REQUIRED)

set(CMAKE_BUILD_TYPE Release)

##########################
## Declare ROS messages ##
##########################

# add_message_files(
#   FILES
#   EgoTwist2DUnicycle.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES invdyn_controller
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  DEPENDS Eigen
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(lqr_controller src/lqr.cpp src/main_lqr.cpp )

add_dependencies(lqr_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(lqr_controller
${catkin_LIBRARIES} 
)

add_executable(pitch_loop src/main_pitch_loop.cpp )

add_dependencies(pitch_loop ${catkin_EXPORTED_TARGETS})

target_link_libraries(pitch_loop
${catkin_LIBRARIES} 
)

