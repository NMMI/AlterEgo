cmake_minimum_required(VERSION 3.10)

project(alterego_visualization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
  sensor_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS /usr/include/eigen3)


set(CMAKE_BUILD_TYPE Release)


catkin_package(
 INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
#  LIBRARIES my_ego
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs alterego_msgs sensor_msgs
#  DEPENDS system_lib
  DEPENDS Eigen3
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)


add_executable(visualize_robot_node 
src/visualize_robot.cpp 
src/main_visualize_robot.cpp
)
add_dependencies(visualize_robot_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(visualize_robot_node
  ${catkin_LIBRARIES}
  ${RBDL_LIBRARY}
  ${RBDL_URDFReader_LIBRARY}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_ego.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


