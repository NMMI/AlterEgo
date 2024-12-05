cmake_minimum_required(VERSION 3.10)
project(ego_dance)

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
  ego_msgs
)

## Find Eigen3
find_package(Eigen3 REQUIRED)
## Find RBDL
find_package(RBDL COMPONENTS URDFReader REQUIRED)
set(Eigen3_INCLUDE_DIRS /usr/include/eigen3)  # Forza il percorso se necessario

set(CMAKE_BUILD_TYPE Release)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs ego_msgs
  DEPENDS Eigen3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIR}
)

add_executable(dance_with_me_node src/controller.cpp src/dance_with_me.cpp src/main_dance_with_me.cpp)
add_dependencies(dance_with_me_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dance_with_me_node 
    ${catkin_LIBRARIES} 
    ${RBDL_LIBRARY} 
    ${RBDL_URDFReader_LIBRARY}
)
