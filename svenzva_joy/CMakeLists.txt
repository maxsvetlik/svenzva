cmake_minimum_required(VERSION 2.8.3)
project(svenzva_joy)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  roscpp
  actionlib_msgs
  svenzva_msgs
)

catkin_package(
    DEPENDS actionlib_msgs svenzva_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(svenzva_joystick_controller src/svenzva_joystick_controller.cpp)
target_link_libraries(svenzva_joystick_controller ${catkin_LIBRARIES})
add_dependencies(svenzva_joystick_controller svenzva_msgs_generate_messages_cpp)

add_executable(svenzva_2axis_controller src/svenzva_2axis_controller.cpp)
add_dependencies(svenzva_2axis_controller svenzva_msgs_generate_messages_cpp)
target_link_libraries(svenzva_2axis_controller ${catkin_LIBRARIES})
