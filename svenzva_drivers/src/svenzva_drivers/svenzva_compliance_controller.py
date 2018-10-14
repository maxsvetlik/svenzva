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

from __future__ import division
from threading import Thread

import rospy
import actionlib
import math
import pandas as pd
import numpy

from std_msgs.msg import Float64, Int32, Float64MultiArray
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
        self.test_pub = rospy.Publisher("/revel/smoothed_current", Float64)
        self.gr = [4,6,6,3,4,1,1]
        self.smoothed_torque = [0, 0, 0, 0, 0, 0, 0]

        self.model_torque = [0, 0, 0, 0, 0, 0, 0]
        self.last_model_torque = [0, 0, 0, 0, 0, 0, 0]
        self.teaching_mode = teaching_mode
        self.max_current = False
        self.pos_active = False
        self.pre_error = 0
	self.torque_window = list()
        self.motor_boost = 0

    def motor_state_cb(self, data):
        self.last_motor_state = self.motor_state
        self.motor_state = data

	torque_ar = [0 for x in range(7)]
	for i,state in enumerate(data.motor_states):
		torque_ar[i] = state.load
	if len(self.torque_window) >= 6:
		self.torque_window.pop(0)
	self.torque_window.append(torque_ar)

    #returns the current sliding window for the motor index specified
    # where index = motor_id - 1
    def extract_motor_window(self, index):
        window = list()
	for windows in self.torque_window:
		window.append(windows[index])
        return window

    def model_effort_cb(self, msg):
        if not msg:
            return
        self.last_model_torque = self.model_torque
        self.model_torque = msg.effort

    """
    Triangular smoothing function for noisy torque reading
    """
    def smoothListTriangle(self, list_val,strippedXs=False,degree=3):
     weight=[]
     window=degree*2-1
     smoothed=[0.0]*(len(list_val)-window)
     for x in range(1,2*degree):
         weight.append(degree-abs(degree-x))
     	 w=numpy.array(weight)
     for i in range(len(smoothed)):
         smoothed[i]=sum(numpy.array(list_val[i:i+window])*w)/float(sum(w))
     return smoothed

    def pd_smooth(self, ts):
        data = {'score': ts}
        df = pd.DataFrame(data)
        return df.rolling(window=5).mean()
        #smooth_data = pd.rolling_mean(ts,5)
	#return smooth_data

    #current / torque based complaince
    #threshold theoretically helps smoothness
    #and offset makes the joint easier to move due to model errors
    def feel_and_react_motor(self, motor_id, threshold=3, offset=0):
        #filter_thresh = 50 # a delta larger than this value results in a discard
        #delta_pos = -10
        #delta_neg = 10
        model_torque = self.model_torque[motor_id-1] + offset
        #convert from Nm to raw current value
        model_boost = self.get_raw_current(0.02) #self.get_raw_current(model_torque / self.gr[motor_id-1] * 0.01)
        model_torque = self.get_raw_current(model_torque / self.gr[motor_id-1])

        if abs(self.smoothed_torque[motor_id - 1] - model_torque) > threshold:

            mag = int(math.copysign(1, model_torque - self.motor_state.motor_states[motor_id-1].load ))
            goal = model_torque

            if motor_id is 1:
                if mag > 0:
                    goal -= model_boost #* self.gr[motor_id - 1]
                else:
                    goal += model_boost #* self.gr[motor_id - 1]
                    self.motor_boost = 0.02

            return (motor_id, goal)

        if motor_id is 1:
            self.motor_boost /= 2
            goal = self.get_raw_current(self.motor_boost)
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
                    vals.append( (i,1900) )
                self.mx_io.set_multi_current(tuple(vals))
                self.max_current = True
                return

            elif is_moving and self.max_current:
                return

        vals = []
        vals.append(self.feel_and_react_motor(1, 1))
        vals.append(self.feel_and_react_motor(2, 4))
        vals.append(self.feel_and_react_motor(3, 4))
        vals.append(self.feel_and_react_motor(4, 2))
        vals.append(self.feel_and_react_motor(5, 2))
        vals.append(self.feel_and_react_motor(6, 10))
        vals = [x for x in vals if x is not None]

        if len(vals) > 0:
            self.mx_io.set_multi_current(tuple(vals))

        return

    def rad_to_raw(self, angle):
        return int(round( angle * 4096.0 / 6.2831853 ))

    def update_torque(self):
        for i in range(0, 6):
            self.smoothed_torque[i] = self.smoothListTriangle(self.extract_motor_window(i))[0]

    def start(self):
        rospy.sleep(2.0)
	while len(self.torque_window) < 6:
		rospy.sleep(0.1)
        while not rospy.is_shutdown():
            self.update_torque()
            self.feel_and_react()
            rospy.sleep(0.02)

