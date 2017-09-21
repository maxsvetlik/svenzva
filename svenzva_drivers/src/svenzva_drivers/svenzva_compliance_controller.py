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

from __future__ import division
from threading import Thread

import rospy
import actionlib
import math

from std_msgs.msg import Float64, Int32
from svenzva_msgs.msg import MotorState, MotorStateList, GripperFeedback, GripperResult, GripperAction
from sensor_msgs.msg import JointState

class SvenzvaComplianceController():

    """
    Teaching mode should be enabled when the motors are context-switched into force control mode.
        This mode allows full movement and compliance from a human.

    Otherwise, it is assumed the arm is in current-based position control mode, where compliance is
        intelligently turned on while the arm is not carying out a trajectory, or has failed in doing so.

    """
    def __init__(self, controller_namespace, mx_io, teaching_mode=False):
        self.mx_io = mx_io
        self.motor_state = MotorStateList()
        self.last_motor_state = MotorStateList()
        rospy.Subscriber("revel/motor_states", MotorStateList, self.motor_state_cb, queue_size=1)
        rospy.Subscriber("revel/model_efforts", JointState, self.model_effort_cb)
        rospy.Subscriber("revel/motor1/pos_cmd", Float64, self.pos_cmd_cb)
        self.gr = [1,3,3,3,10,1,1]
        self.model_torque = [0, 0, 0, 0, 0, 0, 0]
        self.last_model_torque = [0, 0, 0, 0, 0, 0, 0]
        self.teaching_mode = teaching_mode
        self.max_current = False
        self.pos_cmd = 0
        self.pos_active = False
        self.pre_error = 0

    def motor_state_cb(self, data):
        self.last_motor_state = self.motor_state
        self.motor_state = data


    def model_effort_cb(self, msg):
        if not msg:
            return
        self.last_model_torque = self.model_torque
        self.model_torque = msg.effort

    def pos_cmd_cb(self, msg):
        self.pos_cmd = msg.data
        self.pos_active = True


    #current / torque based complaince
    #threshold theoretically helps smoothness
    #and offset makes the joint easier to move due to model errors
    def feel_and_react_motor(self, motor_id, threshold=3, offset=0):
        filter_thresh = 50 # a delta larger than this value results in a discard
        delta_pos = -50
        delta_neg = 50
        model_torque = self.model_torque[motor_id-1] + offset
        #convert from Nm to raw current value
        if abs(self.model_torque[motor_id-1] - self.last_model_torque[motor_id-1]) > 1:
            return None
        model_torque = self.get_raw_current(model_torque )
        return (motor_id, model_torque)

        """
        if not self.teaching_mode:
            model_torque = model_torque*50
            if 0 <= model_torque <= 10:
                model_torque = 50
            if -10 <= model_torque <= 0:
                model_torque = -50
        """
        #filter out random sign changes in current
        #if motor_id is not 1:
        if self.motor_state.motor_states[motor_id - 1].load > 0 and self.last_motor_state.motor_states[motor_id - 1].load < 0:
            self.last_motor_state = self.motor_state
        elif self.motor_state.motor_states[motor_id - 1].load < 0 and self.last_motor_state.motor_states[motor_id - 1].load > 0:
            self.last_motor_state = self.motor_state
            return

        # temporary for testing RNE

        if abs(self.motor_state.motor_states[motor_id - 1].load - model_torque) > threshold:
            mag = int(math.copysign(1, model_torque - self.motor_state.motor_states[motor_id-1].load ))
            goal = model_torque
            if mag > 0:
                goal += delta_pos #* self.gr[motor_id - 1]
            else:
                goal += delta_neg #* self.gr[motor_id - 1]

            return (motor_id, goal)

        return

    #tau is torque in Nm
    #output is as firmware expects (units of 3.36mA)
    def get_raw_current(self, tau):
        return int(round(tau / 1.083775 / .00336))

    def feel_and_react(self):

        if not self.teaching_mode:

            moving_status = self.mx_io.get_multi_moving_status(range(1,7))
            is_moving = 0
            for status in moving_status:
                is_moving |= status[1] & 0x2

            if is_moving and not self.max_current:

                vals = []
                #set all motor allowable currents to their maximum
                for i in range(1,7):
                    #self.mx_io.set_goal_pwm(i,450)
                    vals.append( (i,1900) )
                self.mx_io.set_multi_current(tuple(vals))
                self.max_current = True
                #rospy.sleep(0.1)
                #for i in range(1,7):
                #    self.mx_io.set_goal_pwm(i, 700)
                return

            elif is_moving and self.max_current:
                return

        vals = []
        vals.append(self.feel_and_react_motor(1, 1))
        vals.append(self.feel_and_react_motor(2, 2))
        vals.append(self.feel_and_react_motor(3, 2))
        vals.append(self.feel_and_react_motor(4, 5))
        vals.append(self.feel_and_react_motor(5, 1))
        vals.append(self.feel_and_react_motor(6, 10))
        vals = [x for x in vals if x is not None]

        if len(vals) > 0:
            self.mx_io.set_multi_current(tuple(vals))

        rospy.sleep(0.1)


        #update the motors that felt forces
        vals = []
        pos = []
        for i, state in enumerate(self.model_torque):

            #if i == 6:
            #    break

            if i == 0:
                vals.append( ( i+1, self.get_raw_current((state+self.pos_cmd) / self.gr[i]) ))
            else:
                vals.append( ( i+1, self.get_raw_current(state / self.gr[i]) ))

        if len(vals) > 0:
            self.mx_io.set_multi_current(tuple(vals))

        self.max_current = False

        rospy.sleep(0.05)
        return
    """
    def pos_controller(self, motor_id):
        error = 0
        torque = 0
        p_gain = 0.004
        d_gain = 0.0005

        if not self.pos_active:
            return 0

        error =  self.motor_state.motor_states[motor_id-1].position - self.pos_cmd

        if abs(error) < 0.01:
            self.pos_active = False

        torque = p_gain * error + d_gain * ((error - self.pre_error / 0.004))
        self.pre_error = error
        #print "sending: " + str(torque)
        return self.get_raw_current(torque)
    """

    def rad_to_raw(self, angle):
        return int(round( angle * 4096.0 / 6.2831853 ))

    def update_state(self):
        pos = []

        for i in range(1,7):
            pos.append( (i, self.rad_to_raw(self.motor_state.motor_states[i-1].position * self.gr[i-1])))
        self.mx_io.set_multi_position(tuple(pos))
        rospy.sleep(0.1)

    def start(self):
        rospy.sleep(1.0)
        self.feel_and_react()
        while not rospy.is_shutdown():
            self.feel_and_react()
