cmake_minimum_required(VERSION 2.8.3)
project(svenzva_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosbag
  geometry_msgs
  message_generation
  moveit_ros_planning_interface
  svenzva_msgs
)

catkin_package( )


add_executable(fkine src/fkine_demo.cpp)
add_dependencies(fkine svenzva_demo_gencpp)
target_link_libraries(fkine ${catkin_LIBRARIES})

