cmake_minimum_required(VERSION 3.0.2)
project(alterego_body_inv_dyn)

# Impostazioni per il C++14
add_definitions(-std=c++14)

# Trova i pacchetti ROS necessari
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  genmsg
  message_generation
  tf2_ros
  alterego_msgs
)

# Trova pacchetti di dipendenze di sistema
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(rostest REQUIRED)

# Forza l'inclusione di Eigen3, se necessario
set(Eigen3_INCLUDE_DIRS /usr/include/eigen3)

# Trova il pacchetto kdl_parser e le librerie necessarie
find_library(OROCOS_KDL orocos-kdl)
set(orocos_kdl_LIBRARIES ${OROCOS_KDL})
set(KDL_PARSER_INCLUDE_DIR "/usr/include/kdl_parser")
INCLUDE_DIRECTORIES(${KDL_PARSER_INCLUDE_DIR})
find_library(KDL_PARSER kdl_parser)

# Dichiarazione del pacchetto catkin
catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  CATKIN_DEPENDS alterego_msgs std_msgs
  DEPENDS Eigen3
)

# Includi le directory di intestazione
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${KDL_PARSER_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Aggiungi l'eseguibile
add_executable(arm_inv_dyn src/grav_inv_dyn.cpp)
add_dependencies(arm_inv_dyn ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Linka le librerie necessarie
target_link_libraries(arm_inv_dyn
  ${catkin_LIBRARIES}
  ${OROCOS_KDL}
  ${KDL_PARSER}
)

# Installazione del pacchetto
# Non è necessario installare gli eseguibili o le librerie in questo esempio,
# ma puoi scommentare le righe se necessario.
# install(TARGETS arm_inv_dyn
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Aggiungi la possibilità di test
# catkin_add_gtest(${PROJECT_NAME}-test test/arms_inv_dyn-test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
# endif()
