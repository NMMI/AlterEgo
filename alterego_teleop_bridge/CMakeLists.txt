cmake_minimum_required(VERSION 3.10)
project(alterego_teleop_bridge)

add_definitions(-std=c++11)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  alterego_msgs
)

find_package(Eigen3 REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pilot_bridge
  CATKIN_DEPENDS roscpp rospy std_msgs alterego_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(inbound_data src/inbound_data.cpp)
target_link_libraries(inbound_data ${catkin_LIBRARIES})
add_dependencies(inbound_data ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(inbound_data_unity src/inbound_data_unity.cpp)
target_link_libraries(inbound_data_unity ${catkin_LIBRARIES})
add_dependencies(inbound_data_unity ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############