cmake_minimum_required(VERSION 3.10)
project(alterego_arms_compliant_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  kdl_parser
  tf
  cmake_modules
  urdf
  alterego_msgs
)

## Find Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS /usr/include/eigen3)  # Forza il percorso se necessario

set(CMAKE_BUILD_TYPE Release)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs alterego_msgs
  DEPENDS Eigen3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(arms_compliant_control_node src/main_arms_compliant_control.cpp src/arms_compliant_control.cpp)
add_dependencies(arms_compliant_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(arms_compliant_control_node 
    ${catkin_LIBRARIES}
    ${Eigen3_LIBRARIES}
)
