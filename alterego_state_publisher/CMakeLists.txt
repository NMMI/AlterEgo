cmake_minimum_required(VERSION 3.0.2)
project(alterego_state_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  qb_interface
  geometry_msgs
  roscpp
  rospy
  std_msgs
  kdl_parser
  tf
  cmake_modules
  urdf
  alterego_msgs
  message_generation
  rostest
)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS /usr/include/eigen3)

set(CMAKE_BUILD_TYPE Release)


catkin_package(
  INCLUDE_DIRS include
  INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS}

  # LIBRARIES my_ego
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs alterego_msgs rostest

  # DEPENDS system_lib
  DEPENDS Eigen3
  DEPENDS qb_interface
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}

)

add_executable(alterego_state_publisher
src/alterego_state_publisher.cpp 
include/callbacks_alterego_state_publisher.cpp 
src/main_alterego_state_publisher.cpp
)
add_dependencies(alterego_state_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(alterego_state_publisher
  ${catkin_LIBRARIES}
  ${RBDL_LIBRARY}
  ${RBDL_URDFReader_LIBRARY}
)



add_library(alterego_state_publisher_lib src/alterego_state_publisher.cpp)
add_dependencies(alterego_state_publisher_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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
catkin_add_gtest(${PROJECT_NAME}-test test/${PROJECT_NAME}-test.cpp)
if(TARGET ${PROJECT_NAME}-test)
  set_target_properties(${PROJECT_NAME} PROPERTIES ENABLE_EXPORTS ON)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} alterego_state_publisher_lib)
endif()
add_dependencies(${PROJECT_NAME}-test ${catkin_EXPORTED_TARGETS})

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


