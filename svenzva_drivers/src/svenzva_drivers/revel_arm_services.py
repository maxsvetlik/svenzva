#!/usr/bin/env python
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
from svenzva_msgs.srv import SetTorqueEnable

class RevelArmServices():


    def __init__(self, controller_namespace, mx_io, num_motors):
        self.mx_io = mx_io
        self.num_motors = num_motors
        self.torque_srv = rospy.Service('/revel/SetTorqueEnable', SetTorqueEnable, self.torque_enable_cb)
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


