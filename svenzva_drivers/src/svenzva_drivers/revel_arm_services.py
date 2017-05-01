#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2017 Svenzva Robotics
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
#  * Neither the name of University of Arizona nor the names of its
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

"""
Svenzva's Revel general utility service interface

Includes exposing the following services:
    + Arm torque enabled / disabled

Author: Maxwell Svetlik
"""
from __future__ import division
from threading import Thread

import rospy
import actionlib

from std_msgs.msg import Float64, Int32
from svenzva_msgs.msg import MotorState, MotorStateList, GripperFeedback, GripperResult, GripperAction
from svenzva_msgs.srv import SetTorqueEnable, HomeArm

class RevelArmServices():


    def __init__(self, controller_namespace, mx_io, num_motors):
        self.mx_io = mx_io
        self.num_motors = num_motors
        self.torque_srv = rospy.Service('/revel/SetTorqueEnable', SetTorqueEnable, self.torque_enable_cb)
        self.home_srv =rospy.Service('home_arm_service', HomeArm, self.home_arm)
        self.start()

    def torque_enable_cb(self, data):
        if len(data.motor_list) !=  self.num_motors:
            rospy.logerr("SetTorqueEnable: input motor_list is not the right length. Aborting.")
            data.success = False
            return
        for i, val in enumerate(data.motor_list):
            if val > 1 or val < 0:
                rospy.logwarn("Torque value for motor %d was not binary. Interpreting as ENABLE.", i)
                val = 1
            self.mx_io.set_torque_enabled(i+1, val)
            rospy.sleep(0.01)


    def start(self):
        rospy.loginfo("Starting RevelArmServices...")
        #rospy.spin()


    def home_arm(self, data):
        # load the yaml file that specifies the home position
        rospack = rospkg.RosPack()
        path = rospack.get_path('svenzva_drivers')
        f = open(path+"/config/home_position.yaml")
        qmap = yaml.safe_load(f)
        f.close()

        #2 - execute action on yaml file
        req = SvenzvaJointGoal()
        if len(qmap['home']) < 6:
            rospy.logerr("Could not home arm. Home position configuration file ill-formed or missing. Aborting.")
            return
        req.positions = qmap['home']

        fkine = actionlib.SimpleActionClient('/svenzva_joint_action', SvenzvaJointAction)

        fkine.wait_for_server()
        rospy.loginfo("Found Trajectory action server")
        rospy.loginfo("Homing arm...")
        fkine.send_goal_and_wait(req)

