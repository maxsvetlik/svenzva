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


import rospy
import rospkg
import actionlib
import yaml
from std_msgs.msg import Bool
from dynamixel_controllers.srv import *
from svenzva_msgs.msg import *
from svenzva_drivers.srv import *
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction,            FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from collections import deque


#sends a j6 command as specified in the given yaml file.
#files must exist in the config directory of the demo package
def js_playback(filename, state_name):
    global qmap, fkine

    #2 - execute action on yaml file
    req = SvenzvaJointGoal()
    if len(qmap[state_name]) < 6:
        rospy.logerr("Could not find specified state. Configuration file ill-formed or missing. Aborting.")
        return
    req.positions = qmap[state_name]

    rospy.loginfo("Sending state command...")
    fkine.send_goal_and_wait(req)


def js_cb(data):
    global joint_states, states_list
    joint_states = data
    states_list.append(data)


def setup():
    rospy.init_node('svenzva_pick_place_demo', anonymous=False)
    global qmap, fkine, joint_states, states_list

    states_list = deque(maxlen=10)

    record = rospy.get_param('record_points', False)
    joint_states = JointState()
    gripper_client = actionlib.SimpleActionClient('/revel/gripper_action', GripperAction)
    joint_states_sub = rospy.Subscriber('/joint_states', JointState, js_cb, queue_size=1)
    fkine = actionlib.SimpleActionClient('/svenzva_joint_action', SvenzvaJointAction)
    fkine.wait_for_server()
    rospy.loginfo("Found Trajectory action server")

    gripper_client.wait_for_server()
    rospy.loginfo("Found Revel gripper action server")

    goal = GripperGoal()

    filename = "full_pick_and_place"
    fname=""
    rospack = rospkg.RosPack()
    path = rospack.get_path('svenzva_demo')

    rospy.sleep(1)
    if record:

        while(not rospy.is_shutdown()):
            raw_input("Press Enter to save the current joint state to file. Enter q/Q for exit.")
            saved_js = joint_states
            name = raw_input("What to name this point: ")
            if name == 'q' or name == 'Q':
                rospy.shutdown()
            f = open(path+"/config/" + filename + ".yaml", "a")
            ar = []
            ar.append(saved_js.position[0])
            ar.append(saved_js.position[1])
            ar.append(saved_js.position[2])
            ar.append(saved_js.position[3])
            ar.append(saved_js.position[4])
            ar.append(saved_js.position[5])
            f.write(name + ": " + str(ar) + "\n")
            f.close()
    else:

        f = open(path+"/config/" + filename + ".yaml")
        qmap = yaml.safe_load(f)
        f.close()

        #go to first position
        js_playback(fname, "home")


        while not rospy.is_shutdown():

            """
            Grab an existing item from location 1
            """
            gripper_client.send_goal(open_gripper())

            js_playback(fname, "retract_p1_2")


            js_playback(fname, "approach_p1_1")
            js_playback(fname, "approach_p1_2")

            js_playback(fname, "grasp_p1")

            gripper_client.send_goal(close_gripper(500))

            js_playback(fname, "retract_p1_1")
            js_playback(fname, "retract_p1_2")

            js_playback(fname, "hand_off")

            rospy.sleep(1.0)
            gripper_client.send_goal(feel_and_open_gripper())

            rospy.sleep(0.5)

            """
            Take an item from human and drop it off at location 2
            """
            rospy.sleep(1.0)

            gripper_client.send_goal(feel_and_close_gripper(200))

            rospy.sleep(0.1)

            js_playback(fname, "approach_p2")

            js_playback(fname, "grasp_p2")

            gripper_client.send_goal(open_gripper())

            rospy.sleep(0.5)

            js_playback(fname, "retract_p2_1")
            js_playback(fname, "retract_p2_2")

            js_playback(fname, "hand_off")

            """
            Take an item from human and take to location 1
            """
            rospy.sleep(1.0)

            gripper_client.send_goal(feel_and_close_gripper(300))


            js_playback(fname, "approach_p1_1")
            js_playback(fname, "grasp_p1")

            gripper_client.send_goal(open_gripper())

            rospy.sleep(0.5)

            js_playback(fname, "retract_p1_1")

            """
            Pick up item from location 2 and hand to human
            """


            js_playback(fname, "approach_p2")

            js_playback(fname, "grasp_p2")

            gripper_client.send_goal(close_gripper(300))

            rospy.sleep(0.5)

            js_playback(fname, "retract_p2_2")

            js_playback(fname, "hand_off")

            rospy.sleep(1.0)

            gripper_client.send_goal(feel_and_open_gripper())

            rospy.sleep(1.0)




        #old stuff. notably, the gripping current was 400 mA for the 1.2kg box. == .7 Nm
        # should delete when above script is working
        """
        goal.target_action = goal.CLOSE
        goal.target_current = 400
        gripper_client.send_goal(goal)
        rospy.sleep(0.5)

        js_playback(fname, "lift1")
        js_playback(fname, "lift2")
        js_playback(fname, "lift1")
        js_playback(fname, "grasp")


        rospy.sleep(0.5)
        goal.target_action = goal.OPEN
        gripper_client.send_goal(goal)
        rospy.sleep(0.5)

        js_playback(fname, "approach")
        js_playback(fname, "home")



        while not rospy.is_shutdown():
            if gripper_react():
                goal.target_action = goal.CLOSE
                goal.target_current = 100
                gripper_client.send_goal(goal)
                break
            rospy.sleep(0.1)

        """

def open_gripper():
    goal = GripperGoal()
    goal.target_action = goal.OPEN
    return goal

def close_gripper(target_current):
    goal = GripperGoal()
    goal.target_action = goal.CLOSE
    goal.target_current = target_current
    return goal

def feel_and_open_gripper():
    while not rospy.is_shutdown():
        if gripper_react():
            return open_gripper()
        rospy.sleep(0.1)


def feel_and_close_gripper(target_current):
    while not rospy.is_shutdown():
        if gripper_react():
            return close_gripper(target_current)
        rospy.sleep(0.1)



"""
Returns whether or not the gripper wrist feels a non-negligible external force
"""
def gripper_react():
    global joint_states, states_list
    delta = 25


    while not rospy.is_shutdown():
        avg = 0
        for state in list(states_list):
            avg = avg + state.effort[5]
            avg = avg / len(states_list)

            if abs(avg - joint_states.effort[5]) > delta:
                return True
    return False

if __name__ == '__main__':
    try:
        setup()
    except rospy.ROSInterruptException:
        pass


