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
Svenzva's Revel Gripper action interface

Opening and closing are performed at fixed torque rates.
The 'target_current' value in the action goal is only used in the event that
'target_action' is 'close'. In which case, after the fingers are touching the target object,
the 'target_current' torque value is applied.


Author: Maxwell Svetlik
"""
from __future__ import division
from threading import Thread

import rospy
import actionlib

from std_msgs.msg import Float64, Int32
from svenzva_msgs.msg import MotorState, MotorStateList, GripperFeedback, GripperResult, GripperAction

class RevelGripperActionServer():

    _feedback = GripperFeedback()
    _result = GripperResult()

    def __init__(self, controller_namespace, mx_io):
        self.mx_io = mx_io
        self.motor_id = 7
        self.closing_force = 10 #mA
        self.opening_force = -30 #mA
        self.moving_distance = 2.5 #rad
        self.motor_state = MotorState()
        rospy.Subscriber("revel/motor_states", MotorStateList, self.motor_state_cb, queue_size=1)

        self._as = actionlib.SimpleActionServer("revel/gripper_action", GripperAction, execute_cb=self.gripper_cb, auto_start = False)
        self._as.start()

    def gripper_cb(self, goal):
        r = rospy.Rate(0.5)
        success = True

        if goal.target_action == goal.OPEN:
            self._feedback.action = 'opening'
        elif goal.target_action == goal.CLOSE:
            self._feedback.action = 'closing'
        else:
            rospy.logerr("Unable to perform gripper action due to malformed command. Expecting [open, close], got: %s", goal.target_action)
            self._result.success = False
            self._as.set_aborted(self._result)
            return

        self._as.publish_feedback(self._feedback)
        rospy.loginfo("Executing gripper action.")

        if goal.target_action == goal.OPEN:
            self.open_gripper(self.opening_force)
        elif goal.target_action == goal.CLOSE:
            #close fingers gently until they have met the target object
            self.close_gripper(self.closing_force)
            rospy.sleep(0.5)
            self._feedback.action = 'grasping'
            self._as.publish_feedback(self._feedback)
            self.close_gripper(goal.target_current)

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                return
        self._result.success = True
        #self._as.set_succeeded(self._result)

    def motor_state_cb(self, data):
        self.motor_state = data.motor_states[self.motor_id - 1]

    def start(self):
        #wait for motor_state
        rospy.sleep(0.2)

        #close gripper completely
        self.mx_io.set_torque_goal(self.motor_id, self.closing_force)

        #wait for fingers to close
        rospy.sleep(3.0)

        #open gripper the initial_distance
        self.initial_pos = self.motor_state.position
        self.mx_io.set_torque_goal(self.motor_id, self.opening_force)

        while( abs(self.initial_pos - self.motor_state.position) < self.moving_distance):
            rospy.sleep(0.05)

        #stop movement
        self.mx_io.set_torque_goal(self.motor_id, 0)


    def open_gripper(self, force):
        cur_pos = 0 #self.motor_state.position
        self.mx_io.set_torque_goal(self.motor_id, force)

        while( abs(self.initial_pos - cur_pos - self.motor_state.position) < self.moving_distance):
            if self._as.is_preempt_requested():
                rospy.loginfo('Gripper action preempted.')
                self._as.set_preempted()
                break
            rospy.sleep(0.05)

        #stop movement
        self.mx_io.set_torque_goal(self.motor_id, 0)


    def close_gripper(self, force):
        self.mx_io.set_torque_goal(self.motor_id, force)


