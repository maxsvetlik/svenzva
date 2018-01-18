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


from cursesmenu import *
from cursesmenu.items import *

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

from os import listdir
from os.path import isfile, join


"""
This class creates a kinesthetic interface for Revel series robotic arms.
Run as a file, this class is used and wrapped around an n-curses style terminal gui.

The KinestheticTeaching class could be imported and used as a base for a drag-and-click GUI,
or voice activated interactive system.
"""

class KinestheticTeaching:

    def __init__(self):

        record = True
        self.joint_states = JointState()
        self.gripper_client = actionlib.SimpleActionClient('/revel/gripper_action', GripperAction)
        joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.js_cb, queue_size=1)
        self.fkine = actionlib.SimpleActionClient('/svenzva_joint_action', SvenzvaJointAction)
        rospy.loginfo("Waiting for fkine trajectory server...")
        self.fkine.wait_for_server()
        rospy.loginfo("Found Trajectory action server")

        rospy.loginfo("Waiting for gripper action server")
        self.gripper_client.wait_for_server()
        rospy.loginfo("Found Revel gripper action server")

        self.gripper_goal = GripperGoal()

        rospack = rospkg.RosPack()
        self.path = rospack.get_path('svenzva_utils')

        self.interaction_name = None
        self.playback_file = None

        #Controls when a given pose will return, when played back on the arm
        # where delta is the maximum deviation of all joints from the target pose array
        self.delta = 0.1


    def js_cb(self, data):
        self.joint_states = data

    def open_gripper(self):
         self.gripper_goal.target_action = self.gripper_goal.OPEN
         self.gripper_client.send_goal(self.gripper_goal)
         rospy.sleep(0.5)
         return

    def close_gripper(self):
        self.gripper_goal.target_action = self.gripper_goal.CLOSE
        self.gripper_goal.target_current = 50
        self.gripper_client.send_goal(self.gripper_goal)
        rospy.sleep(0.5)
        return

    """
    *
    * RECORDING
    *
    * To be called each time the user wishes to save a pose
    """
    def record_state_interaction(self):
        if self.interaction_name is None:
            raw_input("You must set the interaction before saving poses. Press Enter to continue.")
            return

        try:
            f = open(self.path+"/config/" + self.interaction_name + ".yaml", "a+")

            name = raw_input("Name this pose: ")
            ar = []
            ar.append(self.joint_states.position[0])
            ar.append(self.joint_states.position[1])
            ar.append(self.joint_states.position[2])
            ar.append(self.joint_states.position[3])
            ar.append(self.joint_states.position[4])
            ar.append(self.joint_states.position[5])
            f.write(name + ": " + str(ar) + "\n")
            f.close()
        except:
            raw_input("Unable to open file. Path: " + self.path+"/config/"+self.interaction_name+".yaml")
        return

    def record_gripper_interaction(self, open_gripper):
        if self.interaction_name is None:
            raw_input("You must set the interaction before saving poses. Press Enter to continue.")
            return

        try:
            f = open(self.path+"/config/" + self.interaction_name + ".yaml", "a+")

            num_lines = sum(1 for line in f)
            ar = []
            if open_gripper:
                ar.append("open_gripper")
                self.open_gripper()
            else:
                ar.append("close_gripper")
                self.close_gripper()
            f.write("gripper" + num_lines + ": " +str(ar) + "\n")
            f.close()
        except:
            raw_input("Unable to open file. Path: " + self.path+"/config/"+self.interaction_name+".yaml")

        return



    def set_new_interaction_name(self):
        #check if the input filename contains only valid characters
        self.interaction_name = raw_input("Set interaction (file)name: ")
        while not re.match(r'[\w-]*$', self.interaction_name):
            self.interaction_name = raw_input("Set interaction (file)name: ")
        return

    """
    *
    * PLAYBACK
    *
    """

    """
    Sends a j6 or gripper command given a yaml file.
    Stand-alone method for playing a state.
    """
    def js_playback(self, filename, state_name):
        try:
            f = open(self.path+"/config/" + filename + ".yaml")
            qmap = yaml.safe_load(f)
            f.close()
        except:
            rospy.logerr("Could not find specified state file. Does it exist?")
            return

        req = SvenzvaJointGoal()
        if qmap[state_name][0] == "open_gripper":
            self.open_gripper()
        elif qmap[state_name][0] == "close_gripper":
            self.close_gripper()
        else:
            if len(qmap[state_name]) < 6:
                rospy.logerr("Could not find specified state. Configuration file ill-formed or missing. Aborting.")
                return
            req.positions = qmap[state_name]

            rospy.loginfo("Sending state command...")
            self.fkine.send_goal_and_wait(req)

    """
    Plays back a specific state name. Helper fuction
    """
    def js_playback(self, qmap, state_name):

        req = SvenzvaJointGoal()

        #action_name is at 0th index. additional indicies reserved for gripper action specifications- e.g. torque
        if qmap[state_name][0] == "open_gripper":
            self.open_gripper()
        elif qmap[state_name][0] == "close_gripper":
            self.close_gripper()
        else:
            if len(qmap[state_name]) < 6:
                rospy.logerr("Could not find specified state. Configuration file ill-formed or missing. Aborting.")
                return
            req.positions = qmap[state_name]

            rospy.loginfo("Sending state command...")
            self.fkine.send_goal_and_wait(req)


    """
    Plays back an entire interaction- all poses specified in an interaction file
    """
    def playback_interaction(self):
        filename_list = self.get_filelist()
        file_index = SelectionMenu.get_selection(filename_list)

        #check if user exited
        if file_index == len(filename_list):
            return

        filename = filename_list[file_index]
        try:
            f = open(self.path+"/config/" + filename)
            qmap = yaml.safe_load(f)
            f.close()
        except:
            rospy.logerr("Could not find specified state file. Does it exist?")
            raw_input("Could not find specified state file.")
            return

        for state in qmap:
            self.js_playback(qmap, state)
            self.wait_for_stall(qmap[state])

    # This method spins until a stall condition is detected.
    # A stall condition happens when the joint states published from one time step to another have a
    # total change less than DELTA, or
    # when the error value (|target - actual|) of the joint value is below DELTA
    def wait_for_stall(self, q_ar):
        time = rospy.get_rostime()
        max_time = rospy.Duration(10.0)

        while rospy.get_rostime() < time + max_time:
            stalled = True
            for i in range(0,6):
                if abs(self.joint_states.position[i] - q_ar[i]) > self.delta:
                    stalled = False
                    break
            if stalled:
                rospy.loginfo("Arm reached target position.")
                return
            rospy.sleep(10)
        rospy.loginfo("Arm stalled: did not reach target position after 10 seconds.")
        return


    def get_filelist(self):
        mypath = self.path +"/config/"
        return [f for f in listdir(mypath) if isfile(join(mypath, f))]

    def start_console_menu(self):
        # Create the menu
        menu = CursesMenu("Main", "Teach or playback a guided interaction")

        gripper_menu = CursesMenu("Gripper Interaction", "Teaching a new guided interaction")
        record_menu = CursesMenu("Record Interaction", "Teaching a new guided interaction")

        record_submenu_item = SubmenuItem("Record a new interaction", record_menu, menu)
        gripper_submenu_item = SubmenuItem("Set gripper action", gripper_menu, record_menu)


        # A FunctionItem runs a Python function when selected
        save_pose_item = FunctionItem("Save robot pose", self.record_state_interaction)
        save_gripper_open_item = FunctionItem("Open gripper", self.record_gripper_interaction, [True])
        save_gripper_close_item = FunctionItem("Close gripper", self.record_gripper_interaction, [False])

        playback_item = FunctionItem("Playback an existing interaction", self.playback_interaction)
        set_int_name_item = FunctionItem("Set interaction name", self.set_new_interaction_name)


        gripper_menu.append_item(save_gripper_open_item)
        gripper_menu.append_item(save_gripper_close_item)


        record_menu.append_item(set_int_name_item)
        record_menu.append_item(save_pose_item)
        record_menu.append_item(gripper_submenu_item)

        menu.append_item(record_submenu_item)
        menu.append_item(playback_item)

        # Finally, we call show to show the menu and allow the user to interact
        menu.show()

if __name__ == '__main__':
    #setup_console_menu()
    rospy.init_node('svenzva_kinesthic_teaching_console', anonymous=False)
    try:
        kt = KinestheticTeaching()
        kt.start_console_menu()
    except rospy.ROSInterruptException:
        pass



