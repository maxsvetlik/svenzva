cmake_minimum_required(VERSION 2.8.3)
project(svenzva_drivers)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    std_msgs
    message_generation
    actionlib_msgs
    geometry_msgs
    svenzva_msgs
    kdl_parser
    cmake_modules
    roslib
    sensor_msgs
    hardware_interface
    controller_manager
)

#kdl is cmake only. Must be separte
#find_package( orocos_kdl REQUIRED)
find_package( Eigen REQUIRED)


catkin_python_setup()

generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
)

catkin_package(
    DEPENDS actionlib_msgs orocos_kdl roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(svenzva_dynamics src/svenzva_drivers/svenzva_dynamics.cpp)
target_link_libraries(svenzva_dynamics ${catkin_LIBRARIES})

add_executable(svenzva_cartesian src/svenzva_drivers/revel_cartesian_vel.cpp)
add_dependencies(svenzva_cartesian svenzva_msgs_generate_messages_cpp)
target_link_libraries(svenzva_cartesian ${catkin_LIBRARIES})

add_executable(dynamixel_control_hw src/ros_control/dynamixel_control_hw.cpp src/ros_control/dynamixel_loop.cpp src/ros_control/dynamixel_hardware_interface.cpp)
target_link_libraries(dynamixel_control_hw ${catkin_LIBRARIES})
