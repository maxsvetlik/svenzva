#!/usr/bin/env python
"""
Author: Maxwell Svetlik
"""

from __future__ import division
from threading import Thread

import rospy
import actionlib
import math

from std_msgs.msg import Float64, Int32
from svenzva_msgs.msg import MotorState, MotorStateList, GripperFeedback, GripperResult, GripperAction
from sensor_msgs.msg import JointState

class SvenzvaComplianceController():

    def __init__(self, controller_namespace, mx_io):
        self.mx_io = mx_io
        self.motor_state = MotorStateList()
        rospy.Subscriber("revel/motor_states", MotorStateList, self.motor_state_cb, queue_size=1)
        rospy.Subscriber("revel/model_efforts", JointState, self.model_effort_cb)
        self.model_torque = [0, 0, 0, 0, 0, 0, 0]
        self.real_torque = 0.0
        self.last_felt = 0.0
    def motor_state_cb(self, data):
         self.motor_state = data


    def model_effort_cb(self, msg):
        if not msg:
            return
        self.model_torque = msg.effort


    #position based compliance:
    #doesn't really work well at all
    def feel_and_react_pos(self):
        #if self.motor_state.moving:
        #    return
        threshold = 20 #int(round(1.5 * self.model_torque / 1.083775 / .00336))
        delta = 400
        goal_torque = self.model_torque
        #convert from Nm to raw current value
        goal_torque = int(round(goal_torque / 1.083775 / .00336))

        rospy.loginfo("Joint 4 felt torque: %d", self.motor_state.load)
        rospy.loginfo("Joint 4 model torque: %d", goal_torque)

        if abs(self.motor_state.load - goal_torque) > threshold:
            mag = int(math.copysign(1, goal_torque - self.motor_state.load ))
            rospy.loginfo("Moving to: %d", self.rad_to_raw(self.motor_state.position * 4) + (mag * delta))
            self.mx_io.set_position(4, self.rad_to_raw(self.motor_state.position*4) + (mag * delta))


    #current / torque based complaince
    def feel_and_react_motor(self, motor_id, threshold):
        #threshold = 3 #int(round(1.5 * self.model_torque / 1.083775 / .00336))
        delta_pos = 70
        delta_neg = -65
        goal_torque = self.model_torque[motor_id-1]
        #convert from Nm to raw current value
        goal_torque = int(round(goal_torque / 1.083775 / .00336))

        #rospy.loginfo("Joint %d felt torque: %d", motor_id, self.motor_state.motor_states[motor_id - 1].load)
        #rospy.loginfo("Joint %d model torque: %d", motor_id, goal_torque)


        if abs(self.motor_state.motor_states[motor_id - 1].load - goal_torque) > threshold:
            mag = int(math.copysign(1, goal_torque - self.motor_state.motor_states[motor_id-1].load ))
            goal = goal_torque
            if mag > 0:
                goal += delta_pos
            else:
                goal += delta_neg
            #rospy.loginfo("Moving to: %d", self.rad_to_raw(self.motor_state.position * 4) + (mag * delta))
            #rospy.loginfo("Setting target current to: %d", goal)
            self.mx_io.set_goal_current(motor_id, goal)
            rospy.sleep(0.02)
            self.mx_io.set_goal_current(motor_id, goal_torque)

    def feel_and_react(self):
        self.feel_and_react_motor(1, 3)
        self.feel_and_react_motor(2, 3)
        self.feel_and_react_motor(3, 3)
        self.feel_and_react_motor(4, 3)
        self.feel_and_react_motor(5, 1)
        self.feel_and_react_motor(6, 1)
        return

    def rad_to_raw(self, angle):
        return int(round( angle * 4096.0 / 6.2831853 ))

    def compare(self):
        #gear ratio
        goal_torque = self.model_torque
        #convert from Nm to raw current value
        goal_torque = int(round(goal_torque / 1.083775 / .00336))
        #self.dxl_io.set_goal_current(5, goal_torque)
        rospy.loginfo("Joint 4 felt torque: %f", self.motor_state.motor_states[3].load)
        rospy.loginfo("Joint 4 model torque: %d", goal_torque)
        rospy.sleep(0.1)


    def start(self, rate):
        rospy.sleep(1.0)
        while not rospy.is_shutdown():
            self.feel_and_react()
            rospy.sleep(0.05)
