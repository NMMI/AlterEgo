cmake_minimum_required(VERSION 3.1.3)
project(alterego_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  alterego_msgs
  geometry_msgs
  message_generation
  rospy
  std_msgs
)

add_service_files(
  FILES
  PickService.srv
  HappyPoseService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS alterego_msgs message_runtime geometry_msgs rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/fake_HappyPoseService.py
  src/mainPickObject.py
  src/PickObjectClass.py
  src/HappyPoseService.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})