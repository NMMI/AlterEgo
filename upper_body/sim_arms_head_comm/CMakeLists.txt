cmake_minimum_required(VERSION 3.10)
project(sim_arms_head_comm)

add_definitions(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy tf std_msgs genmsg message_generation)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

find_library(OROCOS_KDL orocos-kdl)
set(orocos_kdl_LIBRARIES ${OROCOS_KDL})

## Declare ROS messages and services
#add_message_files(FILES)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR} ${qb_device_msgs_INCLUDE_DIR})


add_executable(sim_qb_manager src/sim_qb_manager.cpp)
target_link_libraries(sim_qb_manager ${catkin_LIBRARIES} ${OROCOS_KDL})
add_dependencies(sim_qb_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
