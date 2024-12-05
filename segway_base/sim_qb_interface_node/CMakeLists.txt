cmake_minimum_required(VERSION 3.10)
project(sim_qb_interface)

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
  qb_interface
  ego_msgs
)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS /usr/include/eigen3)
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
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
#  LIBRARIES invdyn_controller
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  DEPENDS Eigen3
  DEPENDS qb_interface
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(Sensor_node_sim src/sim_Sensor.cpp src/main_sim_sensor.cpp) 
target_link_libraries(Sensor_node_sim ${catkin_LIBRARIES})
add_dependencies(Sensor_node_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(sim_qb_class_wheels src/sim_qb_class_wheels.cpp src/main_sim_qb_class_wheels.cpp) 
target_link_libraries(sim_qb_class_wheels ${catkin_LIBRARIES})
add_dependencies(sim_qb_class_wheels ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})