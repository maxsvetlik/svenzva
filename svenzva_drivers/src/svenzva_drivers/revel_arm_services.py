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
#  * Neither the name of Svenzva Robotics LLC nor the names of its
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
    + Finger position reset (finger-removal service)
    + Finger hotswap (finger-insert service)
Author: Maxwell Svetlik
"""
from __future__ import division
from threading import Thread

import rospy
import actionlib
import rospkg
import yaml

from std_srvs.srv import Empty
from std_msgs.msg import Float64, Int32
from svenzva_msgs.msg import MotorState, MotorStateList, GripperFeedback, GripperResult, GripperAction

from svenzva_msgs.srv import SetTorqueEnable, HomeArm, SetControlMode


class RevelArmServices():


    def __init__(self, controller_namespace, mx_io, num_motors):
        self.mx_io = mx_io
        self.num_motors = num_motors
        self.gripper_id = 7
        self.torque_srv = rospy.Service('/revel/SetControlMode', SetControlMode, self.control_mode_cb)
        self.torque_srv = rospy.Service('/revel/SetTorqueEnable', SetTorqueEnable, self.torque_enable_cb)
        self.home_srv =rospy.Service('home_arm_service', HomeArm, self.home_arm)
        self.remove_fingers = rospy.Service('/revel/gripper/remove_fingers', Empty, self.remove_fingers_cb)
        self.insert_fingers = rospy.Service('/revel/gripper/insert_fingers', Empty, self.insert_fingers_cb)
        self.motor_state = MotorState()
        rospy.Subscriber("revel/motor_states", MotorStateList, self.motor_state_cb, queue_size=1)

        self.reset_pos = -3.4

    def motor_state_cb(self, data):
        if self.num_motors >= self.gripper_id:
            self.motor_state = data.motor_states[self.gripper_id - 1]


    """
    Service call back that pushes fingers out of gripper and resets motor position for correct finger
    position tracking.
    """
    def remove_fingers_cb(self, data):
        force = 25
        if self.num_motors >= self.gripper_id:
            cur_pos = self.motor_state.position

            #switch to position mode for simplicity
            self.mx_io.set_torque_enabled(7, 0)
            self.mx_io.set_torque_enabled(7, 1)

            self.mx_io.set_position(7, int(round(self.reset_pos * 4096.0 / 6.2831853)))
            rospy.sleep(5.0)

            #switch back to torque mode
            self.mx_io.set_torque_enabled(7, 0)
            self.mx_io.set_operation_mode(7, 0)
            self.mx_io.set_torque_enabled(7, 1)

        return

    """
    Service used in conjunction with `remove_fingers` service. This service should be called after the
    fingers have been pushed into the gripper by the user.
    """
    def insert_fingers_cb(self, data):
        if self.num_motors >= self.gripper_id:
            self.mx_io.set_torque_enabled(7, 0)
            self.mx_io.set_operation_mode(7, 4)
            self.mx_io.set_torque_enabled(7, 1)

            self.mx_io.set_position(7, int(round(0 * 4096.0 / 6.2831853)))
            rospy.sleep(5.0)


            self.mx_io.set_torque_enabled(7, 0)
            self.mx_io.set_operation_mode(7, 0)
            self.mx_io.set_torque_enabled(7, 1)

        return

    def control_mode_cb(self, data):
        tup_list_op = tuple()
        if data.control_mode == data.POSITION:
            tup_list_op = tuple(((1,5),(2,5),(3,5),(4,5),(5,5),(6,5),(7,0)))
        elif data.control_mode == data.VELOCITY:
            tup_list_op = tuple(((1,1),(2,1),(3,1),(4,1),(5,1),(6,1),(7,0)))
        elif data.control_mode == data.TORQUE:
            tup_list_op = tuple(((1,0),(2,0),(3,0),(4,0),(5,0),(6,0),(7,0)))
        else:
            tup_list_op = tuple(((1,5),(2,5),(3,5),(4,5),(5,5),(6,5),(7,0)))
        tup_list_dis = tuple(((1,0),(2,0),(3,0),(4,0),(5,0),(6,0),(7,0)))
        tup_list_en = tuple(((1,1),(2,1),(3,1),(4,1),(5,1),(6,1),(7,1)))
        self.mx_io.sync_set_torque_enabled(tup_list_dis)
        self.mx_io.sync_set_operation_mode(tup_list_op)
        self.mx_io.sync_set_torque_enabled(tup_list_en)
	self.reset_motor_parmeters()
        return True

    def torque_enable_cb(self, data):
        if len(data.motor_list) !=  self.num_motors:
            rospy.logerr("SetTorqueEnable: input motor_list is not the right length. Aborting.")
            #data.success = False
            return
        for i, val in enumerate(data.motor_list):
            if val > 1 or val < 0:
                rospy.logwarn("Torque value for motor %d was not binary. Interpreting as ENABLE.", i)
                val = 1
            self.mx_io.set_torque_enabled(i+1, val)
            rospy.sleep(0.01)
	self.reset_motor_parmeters()
    def torque_goal_cb(self, data):
        if len(data.data) != self.num_motors:
            rospy.logerr("SetTorqueEnable: input motor_list is not the right length. Aborting.")
            return
        cmds = []
        for i, val in enumerate(data.data):
            cmds.append( (i+1, val))
        self.mx_io.set_multi_current(tuple(cmds))
        rospy.sleep(0.01)

    def start(self):
        rospy.loginfo("Starting RevelArmServices...")
        rospy.spin()

    """
       Calls a service that restores the control parameters for each motor, which get reset to defaults when torque is
	disabled, which happes when a user changes modes or dis/enables torque
    """
    def reset_motor_parmeters(self):
	try:
	    reset_motor_param = rospy.ServiceProxy('/revel/reload_firmware_parameters', Empty)
	    return reset_motor_param()
	except rospy.ServiceException, e:
	    rospy.loginfo("Unable to reset motor parmeters after context change... call failed: %s"%e)


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

