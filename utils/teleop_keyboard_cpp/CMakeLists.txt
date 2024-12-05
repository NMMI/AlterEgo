cmake_minimum_required(VERSION 3.10)

project(teleop_keyboard_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  ego_msgs
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp geometry_msgs ego_msgs
)

include_directories(${catkin_INCLUDE_DIRS})



add_executable(teleop_ego_keyboard src/teleop_ego_keyboard.cpp)
add_dependencies(teleop_ego_keyboard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleop_ego_keyboard ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
