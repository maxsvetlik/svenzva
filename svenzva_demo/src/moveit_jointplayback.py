#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
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
import rospkg
import yaml
import actionlib

from std_msgs.msg import String
from svenzva_msgs.msg import *

class JointPlayBack:

    def __init__(self, filename):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("svenzva_arm")
        self.diaply_pub = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self.gripper_client = actionlib.SimpleActionClient('/revel/gripper_action', GripperAction)

        self.setup_filesys(filename)

        self.gripper_client.wait_for_server()
        rospy.loginfo("Found Revel gripper action server")


    def print_info(self):

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        ##
        ## We can get the name of the reference frame for this robot
        print "============ Reference frame: %s" % self.group.get_planning_frame()

        ## We can also print the name of the end-effector link for this group
        print "============ Reference frame: %s" % self.group.get_end_effector_link()

        ## We can get a list of all the groups in the robot
        print "============ Robot Groups:"
        print self.robot.get_group_names()

        ## Sometimes for debugging it is useful to print the entire state of the
        ## robot.
        print "============ Printing robot state"
        print self.robot.get_current_state()
        print "============"


    def setup_filesys(self, filename):
        rospack = rospkg.RosPack()
        path = rospack.get_path('svenzva_demo')
        # load the yaml file that specifies the home position

        f = open(path+"/config/" + filename + ".yaml")
        self.qmap = yaml.safe_load(f)
        f.close()


    def joint_playback(self, state_name):
        self.group.clear_pose_targets()

        ## Then, we will get the current set of joint values for the group
        group_variable_values = self.group.get_current_joint_values()
        print "============ Joint values: ", group_variable_values

        for i,val in enumerate(self.qmap[state_name]):
            group_variable_values[i] = val

        self.group.set_joint_value_target(group_variable_values)
        plan2 = self.group.plan()
        self.group.go(wait=True)

    def quit(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("STOPPING playback")


if __name__=='__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.0)
    rospy.init_node('svenzva_moveit_joint_playback', anonymous=True)

    playback = JointPlayBack("book")

    goal = GripperGoal()

    #open gripper
    goal.target_action = goal.OPEN
    playback.gripper_client.send_goal(goal)
    rospy.sleep(0.5)

    playback.joint_playback("start1")
    rospy.sleep(3.0)
    playback.joint_playback("start2")
    rospy.sleep(3.0)
    playback.joint_playback("approach3")
    rospy.sleep(3.0)
    playback.joint_playback("reach4")
    rospy.sleep(3.0)

    goal.target_action = goal.CLOSE
    goal.target_current = 700
    playback.gripper_client.send_goal(goal)
    rospy.sleep(2.0)

    playback.joint_playback("grab5")
    rospy.sleep(3.0)



    playback.joint_playback("pullback6")

    rospy.sleep(2.0)
    playback.joint_playback("turn7")
    rospy.sleep(2.0)
    playback.joint_playback("move8")
    rospy.sleep(2.0)

    #open gripper
    goal.target_action = goal.OPEN
    playback.gripper_client.send_goal(goal)
    rospy.sleep(0.5)

    playback.quit()
