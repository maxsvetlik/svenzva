#!/usr/bin/env python
"""
Svenzva's Revel gripper interface
"""
from __future__ import division
from threading import Thread

import rospy
import actionlib

from std_msgs.msg import Float64, Int32
from svenzva_msgs.msg import MotorState, MotorStateList

class RevelGripperActionServer():
    def __init__(self, controller_namespace, mx_io):
        self.mx_io = mx_io
        self.motor_id = 7
        self.initial_force = 5 #newtons
        self.moving_distance = 2.8 #rad
        self.motor_state = MotorState()
        rospy.Subscriber("revel/motor_states", MotorStateList, self.motor_state_cb, queue_size=1)
        rospy.Subscriber("revel/open_gripper", Int32, self.open_cb)

    def open_cb(self, data):
        if data.data == 0:
            self.close_gripper(10)
        elif data.data == -1:
            self.close_gripper(50)
        elif data.data == 1:
            self.open_gripper(-20)

        rospy.sleep(0.05)

    def motor_state_cb(self, data):
        self.motor_state = data.motor_states[self.motor_id - 1]

    def start(self):
        #wait for motor_state
        rospy.sleep(0.2)

        #close gripper completely
        self.mx_io.set_torque_goal(self.motor_id, 10)

        #wait for fingers to close
        rospy.sleep(3.0)

        #open gripper the initial_distance
        self.initial_pos = self.motor_state.position
        self.mx_io.set_torque_goal(self.motor_id, -20)

        while( abs(self.initial_pos - self.motor_state.position) < self.moving_distance):
            rospy.sleep(0.05)

        #stop movement
        self.mx_io.set_torque_goal(self.motor_id, 0)

        rospy.spin()

    def open_gripper(self, force):
        cur_pos = 0 #self.motor_state.position
        self.mx_io.set_torque_goal(self.motor_id, force)

        while( abs(self.initial_pos - cur_pos - self.motor_state.position) < self.moving_distance):
            rospy.sleep(0.05)

        #stop movement
        self.mx_io.set_torque_goal(self.motor_id, 0)


    def close_gripper(self, force):
        self.mx_io.set_torque_goal(self.motor_id, force)


