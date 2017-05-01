cmake_minimum_required(VERSION 2.8.3)
project(svenzva_msgs)

find_package(catkin REQUIRED
  COMPONENTS message_generation std_msgs actionlib)

add_message_files(
  FILES
  JointState.msg
  MotorState.msg
  MotorStateList.msg
)

add_action_files(DIRECTORY action FILES Gripper.action SvenzvaJoint.action)

add_service_files(DIRECTORY srv FILES SetTorqueEnable.srv)

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs actionlib
)
