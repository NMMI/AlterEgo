cmake_minimum_required(VERSION 2.8.3)
project(ego_model)


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

FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)
FIND_PACKAGE (Eigen3 REQUIRED)
find_package(Eigen REQUIRED)

set(CMAKE_BUILD_TYPE Release)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES EgoModel
  CATKIN_DEPENDS roscpp rospy std_msgs
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

add_executable(ego_model src/EgoModel.cpp src/main_ego.cpp)

add_dependencies(ego_model ${catkin_EXPORTED_TARGETS})

target_link_libraries(ego_model 
  ${catkin_LIBRARIES} 
  ${RBDL_LIBRARY}
  ${RBDL_URDFReader_LIBRARY}
)
