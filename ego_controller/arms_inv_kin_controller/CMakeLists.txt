cmake_minimum_required(VERSION 2.8.3)
project(arms_inv_kin_controller)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  kdl_conversions
  roscpp
  tf
  rospy
  std_msgs
  rosbag
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES arms_inv_kin
 CATKIN_DEPENDS 
 	kdl_parser 
 	roscpp 
 	rospy 
 	std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/arms_inv_kin
  include/utils
)

##############################
## Declare a C++ executable ##
##############################

add_executable(only_gravity_comp_node src/main_only_gravity_comp.cpp src/only_gravity_comp.cpp )
add_dependencies(only_gravity_comp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(only_gravity_comp_node ${catkin_LIBRARIES})

add_executable(inv_kin_gravity_comp_node src/main_inv_kin_gravity_comp.cpp src/inv_kin_gravity_comp.cpp )
add_dependencies(inv_kin_gravity_comp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inv_kin_gravity_comp_node ${catkin_LIBRARIES})

