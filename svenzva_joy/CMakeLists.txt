cmake_minimum_required(VERSION 2.8.3)
project(svenzva_joy)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  roscpp
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(svenzva_joystick_controller src/svenzva_joystick_controller.cpp)
target_link_libraries(svenzva_joystick_controller ${catkin_LIBRARIES})
