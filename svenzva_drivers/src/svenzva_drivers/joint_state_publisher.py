#!/usr/bin/env python

"""
    joint_state_publisher              - Version 2.0 2017-3-18
    dynamixel_joint_state_publisher.py - Version 1.0 2010-12-28

    Publish the dynamixel_controller joint states on the /joint_states topic

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.
    Copyright (c) 2017 Svenzva Robotics

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import roslib
import rospy

from sensor_msgs.msg import JointState as JointStatePR2
from svenzva_msgs.msg import MotorStateList

class JointStateMessage():
    def __init__(self, name, position, velocity, effort):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort

class JointStatePublisher():
    def __init__(self):
        rospy.init_node('joint_state_publisher', anonymous=True)

        rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(rate)
        namespace = rospy.get_param('~arm_namespace', '')
        self.joints = rospy.get_param('joint_names', '')

        self.motor_states = []

        # Start controller state subscribers
        rospy.Subscriber(namespace + '/motor_states', MotorStateList, self.motor_state_cb)

        # Start publisher
        self.joint_states_pub = rospy.Publisher('/joint_states', JointStatePR2, queue_size=1)

        rospy.loginfo("Starting Joint State Publisher at " + str(rate) + "Hz")

        #2*theta*r / pi = linear_distance due to 1:1 gear ratio for rack/pinion system
        self.finger_divisor = .0246 / 3.14159 * 2

        while not rospy.is_shutdown():
            self.publish_joint_states()
            r.sleep()

    def motor_state_cb(self, msg):
        self.motor_states = msg.motor_states

    def publish_joint_states(self):
        # Construct message & publish joint states
        msg = JointStatePR2()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []

        for i, joint in enumerate(self.motor_states):
            #linear fit into torque in Nm
            effort_nm = joint.load * 0.00336 * 1.083775
            if i == 6:
                #finger 1
                msg.name.append(self.joints[i])
                msg.position.append(-1 * joint.position * self.finger_divisor)
                msg.velocity.append(-1 * joint.speed)
                msg.effort.append(-1 * effort_nm)


                #finger 2
                msg.name.append(self.joints[i+1])
                msg.position.append(1 * joint.position * self.finger_divisor)
                msg.velocity.append(1 * joint.speed)
                msg.effort.append(1 * effort_nm)
            else:
                msg.name.append(self.joints[i])
                msg.position.append(joint.position)
                msg.velocity.append(joint.speed)
                msg.effort.append(effort_nm)

            msg.header.stamp = rospy.Time.now()

        self.joint_states_pub.publish(msg)

if __name__ == '__main__':
    try:
        s = JointStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException: pass

