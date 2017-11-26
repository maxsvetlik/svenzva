#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Svenzva Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf

def move_group_python_interface_tutorial():
  moveit_commander.roscpp_initialize(sys.argv)
  global joy_cmd
  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  scene = moveit_commander.PlanningSceneInterface()

  group = moveit_commander.MoveGroupCommander("svenzva_arm")

  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,
                                      queue_size=20)

  while not rospy.is_shutdown():
      if joy_cmd != geometry_msgs.msg.Twist():
          cur_pose = group.get_current_pose();
          quaternion = cur_pose.pose.orientation
          explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
          euler = tf.transformations.euler_from_quaternion(explicit_quat)
          new_euler = [euler[0] + joy_cmd.angular.x, euler[1] + joy_cmd.angular.y, euler[2] + joy_cmd.angular.z]

          quat = tf.transformations.quaternion_from_euler(new_euler[0], new_euler[1], new_euler[2])
          pose_target = geometry_msgs.msg.Pose()
          pose_target.orientation.x = quat[0]
          pose_target.orientation.y = quat[1]
          pose_target.orientation.z = quat[2]
          pose_target.orientation.w = quat[3]
          pose_target.position.x = cur_pose.pose.position.x + joy_cmd.linear.x
          pose_target.position.y = cur_pose.pose.position.y + joy_cmd.linear.y
          pose_target.position.z = cur_pose.pose.position.z + joy_cmd.linear.z
          group.set_pose_target(pose_target)

          #display_trajectory = moveit_msgs.msg.DisplayTrajectory()
          #display_trajectory.trajectory_start = robot.get_current_state()
          #display_trajectory.trajectory.append(plan1)
          #display_trajectory_publisher.publish(display_trajectory);

          group.go(wait=False)
      rospy.sleep(0.1)

  """
  ## Cartesian Paths
  ## ^^^^^^^^^^^^^^^
  ## You can plan a cartesian path directly by specifying a list of waypoints
  ## for the end-effector to go through.
  waypoints = []

  # start with the current pose
  waypoints.append(group.get_current_pose().pose)

  # first orient gripper and move forward (+x)
  wpose = geometry_msgs.msg.Pose()
  wpose.orientation.w = 1.0
  wpose.position.x = waypoints[0].position.x + 0.1
  wpose.position.y = waypoints[0].position.y
  wpose.position.z = waypoints[0].position.z
  waypoints.append(copy.deepcopy(wpose))

  # second move down
  wpose.position.z -= 0.10
  waypoints.append(copy.deepcopy(wpose))

  # third move to the side
  wpose.position.y += 0.05
  waypoints.append(copy.deepcopy(wpose))

  ## We want the cartesian path to be interpolated at a resolution of 1 cm
  ## which is why we will specify 0.01 as the eef_step in cartesian
  ## translation.  We will specify the jump threshold as 0.0, effectively
  ## disabling it.
  (plan3, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold

  # Uncomment the line below to execute this plan on a real robot.
  # group.execute(plan3)
  """

  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

def joy_callback(data):
  global joy_cmd
  joy_cmd = data

if __name__=='__main__':
  global joy_cmd
  joy_cmd = geometry_msgs.msg.Twist()
  rospy.init_node('collision_aware_cartesian_control', anonymous=True)

  try:
    rospy.Subscriber("/revel/eef_velocity", geometry_msgs.msg.Twist, joy_callback, queue_size=1)
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass


