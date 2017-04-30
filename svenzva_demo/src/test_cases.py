#!/usr/bin/env python

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
    global joint_states
    joint_states = data

def setup():
    rospy.init_node('svenzva_pick_place_demo', anonymous=False)
    global qmap, fkine, joint_states

    record = False
    joint_states = JointState()
    gripper_client = actionlib.SimpleActionClient('/revel/gripper_action', GripperAction)
    joint_states_sub = rospy.Subscriber('/joint_states', JointState, js_cb, queue_size=1)
    fkine = actionlib.SimpleActionClient('/svenzva_joint_action', SvenzvaJointAction)
    fkine.wait_for_server()
    rospy.loginfo("Found Trajectory action server")

    gripper_client.wait_for_server()
    rospy.loginfo("Found Revel gripper action server")

    goal = GripperGoal()

    filename = "repeatability"
    fname=""
    rospack = rospkg.RosPack()
    path = rospack.get_path('svenzva_demo')
    # load the yaml file that specifies the home position

    rospy.sleep(1)
    if record:

        while(not rospy.is_shutdown()):
            raw_input("Press Enter to save the current joint state to file...")
            saved_js = joint_states
            name = raw_input("What to name this point: ")
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

    f = open(path+"/config/" + filename + ".yaml")
    qmap = yaml.safe_load(f)
    f.close()

    #go to first position
    js_playback(fname, "home")

    goal.target_action = goal.CLOSE
    goal.target_current = 40
    gripper_client.send_goal(goal)
    js_playback(fname, "demo_measure2")
    rospy.sleep(5.0)

    js_playback(fname, "home")
    rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        setup()
    except rospy.ROSInterruptException:
        pass


