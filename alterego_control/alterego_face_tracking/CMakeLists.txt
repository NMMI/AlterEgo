cmake_minimum_required(VERSION 2.8.3)
project(alterego_face_tracking)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy geometry_msgs sensor_msgs std_msgs 
)

catkin_install_python(PROGRAMS scripts/face_tracker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
# include 
  ${catkin_INCLUDE_DIRS}
)

# # Add executable
# add_executable(happypose_ros
#   src/happypose_ros.cpp
# )

# # Add dependency for messages generations
# add_dependencies(happypose_ros happypose_ros_generate_messages_cpp)

# target_link_libraries(happypose_ros ${catkin_LIBRARIES})

# # Install the executable
# install(TARGETS happypose_ros
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# # Install generated messages
# install(DIRECTORY ${MESSAGE_DIR}
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
# )

