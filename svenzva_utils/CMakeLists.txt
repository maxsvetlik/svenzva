cmake_minimum_required(VERSION 2.8.3)
project(svenzva_utils)

find_package(catkin REQUIRED COMPONENTS
  mx_msgs
  roscpp
  rospy
  trajectory_msgs
  std_msgs
  sensor_msgs
  svenzva_msgs
  svenzva_drivers
)

# find_package(Boost REQUIRED COMPONENTS system)


catkin_package( )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

