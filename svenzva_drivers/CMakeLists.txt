cmake_minimum_required(VERSION 2.8.3)
project(svenzva_drivers)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    std_msgs
    message_generation
    actionlib_msgs
    geometry_msgs
    svenzva_msgs
    kdl_parser
    orocos_kdl
    cmake_modules
    roslib
    sensor_msgs
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
target_link_libraries(svenzva_cartesian ${catkin_LIBRARIES})
