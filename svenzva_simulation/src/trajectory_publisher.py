#!/usr/bin/env python

"""
This file gives a demonstration of sending a waypoint trajectory programatically.
The trajectory action server can be invoked in two separate ways-
1) Through an actionlib client
2) Through publishing to a topic

Both methods are exemplified here.
"""

import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

def publish_goal():
    action_client = True
    client = actionlib.SimpleActionClient('/revel_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    pub = rospy.Publisher('/revel_trajectory_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('trajectory_tester', anonymous=True)
    traj = JointTrajectory()
    traj.joint_names.append("joint_1")
    traj.joint_names.append("joint_2")
    traj.joint_names.append("joint_3")
    traj.joint_names.append("joint_4")
    traj.joint_names.append("joint_5")
    traj.joint_names.append("joint_6")
    jtp = JointTrajectoryPoint()
    jtp.positions.append(0.0)
    jtp.positions.append(0.0)
    jtp.positions.append(0.0)
    jtp.positions.append(0.0)
    jtp.positions.append(0.0)
    jtp.positions.append(0.0)
    jtp.velocities.append(1.0)
    jtp.velocities.append(1.0)
    jtp.velocities.append(1.0)
    jtp.velocities.append(1.0)
    jtp.velocities.append(1.0)
    jtp.velocities.append(1.0)
    jtp.time_from_start = rospy.Duration(0.0)

    traj.points.append(jtp)
    traj.header.stamp = rospy.Time.now()

    if not action_client:
        pub.publish(traj)
    else:
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        goal.goal_time_tolerance = rospy.Duration(4.0)

        client.wait_for_server()
        print 'Found server'
        client.send_goal(goal)

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass

