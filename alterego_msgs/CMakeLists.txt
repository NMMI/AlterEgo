cmake_minimum_required(VERSION 3.4)
project(alterego_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs sensor_msgs actionlib_msgs)

add_message_files(
  FILES
  EgoArms.msg
  DebugLQR.msg
  UpperBodyState.msg
  LowerBodyState.msg
)

add_service_files(FILES
  NavService.srv
  EnableAutoModeService.srv
  WhereRUService.srv
  LaunchGStreamer.srv
  CloseGStreamer.srv
  WaitPilotService.srv
  HappyPoseService.srv
  PickService.srv
)

# add_action_files(FILES ErrorRecovery.action)

generate_messages(DEPENDENCIES std_msgs sensor_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs sensor_msgs actionlib_msgs)