cmake_minimum_required(VERSION 3.10)
project(arms_head_kin)

add_definitions(-std=c++14)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy tf std_msgs genmsg message_generation tf2_ros ego_msgs) 
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(ego_msgs REQUIRED)

find_library(OROCOS_KDL orocos-kdl)
set(orocos_kdl_LIBRARIES ${OROCOS_KDL})

## Declare ROS messages and services
#add_message_files(FILES)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

# Add the include directory to the include paths
set(KDL_PARSER_INCLUDE_DIR "/usr/include/kdl_parser")
INCLUDE_DIRECTORIES ( ${KDL_PARSER_INCLUDE_DIR})
find_library(KDL_PARSER kdl_parser)
add_library(${PROJECT_NAME}_lib
  src/arm_inv_kin_class.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${qb_device_msgs_INCLUDE_DIR}
)
## Declare a catkin package
catkin_package()


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR} ${qb_device_msgs_INCLUDE_DIR})

add_executable(arm_inv_kin src/arm_inv_kin.cpp)
target_link_libraries(arm_inv_kin ${catkin_LIBRARIES} ${KDL_PARSER} ${OROCOS_KDL})
add_dependencies(arm_inv_kin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(head_inv_kin src/head_inv_kin.cpp)
target_link_libraries(head_inv_kin ${catkin_LIBRARIES} ${OROCOS_KDL})
add_dependencies(head_inv_kin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(pitch_correction src/pitch_correction.cpp)
target_link_libraries(pitch_correction ${catkin_LIBRARIES} ${OROCOS_KDL} ${qb_device_msgs_LIBRARIES})
add_dependencies(pitch_correction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(arm_inv_kin_main src/arm_inv_kin_class.cpp src/arm_inv_kin_main.cpp)
target_link_libraries(arm_inv_kin_main ${catkin_LIBRARIES} ${KDL_PARSER} ${OROCOS_KDL})
add_dependencies(arm_inv_kin_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# add_executable(arm_bridge src/arm_bridge.cpp)
# target_link_libraries(arm_bridge ${catkin_LIBRARIES} ${OROCOS_KDL})
# add_dependencies(arm_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(head_inv_kin_5dof src/head_manager_5dof.cpp)
# target_link_libraries(head_inv_kin_5dof ${catkin_LIBRARIES} ${OROCOS_KDL})
# add_dependencies(head_inv_kin_5dof ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(head_inv_kin_6dof src/head_manager_6dof.cpp)
# target_link_libraries(head_inv_kin_6dof ${catkin_LIBRARIES} ${OROCOS_KDL})
# add_dependencies(head_inv_kin_6dof ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})