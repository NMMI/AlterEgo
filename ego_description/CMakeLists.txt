cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)
project(ego_description)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  urdf
  kdl_parser
)

FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)
FIND_PACKAGE (Eigen3 REQUIRED)

set(CMAKE_BUILD_TYPE Release)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ego_description
#  CATKIN_DEPENDS roscpp rospy std_msgs urdf
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

#############
## Testing ##
#############


#Add_executable(ego_parser src/ego_parser.cpp)
#Add_executable(ego_parser src/teleop_publisher.cpp)
#
#target_link_libraries(ego_parser 
#	${catkin_LIBRARIES}
#	${RBDL_LIBRARY}
#	${RBDL_URDFReader_LIBRARY} )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

