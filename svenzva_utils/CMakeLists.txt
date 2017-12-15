cmake_minimum_required(VERSION 2.8.3)
project(svenzva_utils)

find_package(catkin REQUIRED COMPONENTS
  mx_msgs
  roscpp
  rospy
  svenzva_msgs
)

# find_package(Boost REQUIRED COMPONENTS system)


catkin_package( )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

