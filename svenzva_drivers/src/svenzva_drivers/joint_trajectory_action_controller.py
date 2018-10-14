#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2017 Svenzva Robotics, 2010-2011, Antons Rebguns.
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
#  * Neither the name of University of Arizona, Svenzva Robotics LLC,
#    nor the names of its contributors may be used to endorse or
#    promote products derived from this software without specific
#    prior written permission.
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

from std_msgs.msg import Float64
from svenzva_msgs.msg import MotorState, MotorStateList
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from svenzva_drivers.svenzva_driver import SvenzvaDriver

class Segment():
    def __init__(self, num_joints):
        self.start_time = 0.0  # trajectory segment start time
        self.duration = 0.0  # trajectory segment duration
        self.positions = [0.0] * num_joints
        self.velocities = [0.0] * num_joints

class JointTrajectoryActionController():
    def __init__(self, controller_namespace, mx_io, states):
        #set default values
        self.update_rate = 1000
        self.state_update_rate = 50
        self.trajectory = []

        self.mx_io = mx_io
        self.num_joints = 6
        self.controller_namespace = controller_namespace

        self.joint_names = rospy.get_param('joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])

        #trajectory controller does not control finger movements
        #if finger joints specified, remove them
        if 'finger_joint_1' in self.joint_names:
            self.joint_names.remove('finger_joint_1')
        if 'finger_joint_2' in self.joint_names:
            self.joint_names.remove('finger_joint_2')

        rospy.Subscriber("revel/motor_states", MotorStateList, self.motor_state_cb, queue_size=1)

        self.motor_states = []
        self.joint_to_id = dict(zip(self.joint_names, range(1, self.num_joints+1)))
        self.gear_ratios = [4,7,7,3,4,1]
        self.num_joints = len(self.joint_names)
        self.initialize()

    def initialize(self):
        ns = self.controller_namespace + '/joint_trajectory_action_node/constraints'
        self.goal_time_constraint = rospy.get_param(ns + '/goal_time', 0.0)
        self.stopped_velocity_tolerance = rospy.get_param(ns + '/stopped_velocity_tolerance', 0.01)
        self.goal_constraints = []
        self.trajectory_constraints = []
        self.min_velocity = rospy.get_param(self.controller_namespace + '/joint_trajectory_action_node/min_velocity', 0.1)

        for joint in self.joint_names:
            self.goal_constraints.append(rospy.get_param(ns + '/' + joint + '/goal', -1.0))
            self.trajectory_constraints.append(rospy.get_param(ns + '/' + joint + '/trajectory', -1.0))

        # Message containing current state for all controlled joints
        self.msg = FollowJointTrajectoryFeedback()
        self.msg.joint_names = self.joint_names
        self.msg.desired.positions = [0.0] * self.num_joints
        self.msg.desired.velocities = [0.0] * self.num_joints
        self.msg.desired.accelerations = [0.0] * self.num_joints
        self.msg.actual.positions = [0.0] * self.num_joints
        self.msg.actual.velocities = [0.0] * self.num_joints
        self.msg.error.positions = [0.0] * self.num_joints
        self.msg.error.velocities = [0.0] * self.num_joints

        return True

    def motor_state_cb(self, data):
        self.motor_states = data.motor_states

    def start(self):
        self.running = True

        self.command_sub = rospy.Subscriber(self.controller_namespace + '/command', JointTrajectory, self.process_command)
        self.state_pub = rospy.Publisher(self.controller_namespace + '/state', FollowJointTrajectoryFeedback, queue_size=1)
        self.action_server = actionlib.SimpleActionServer(self.controller_namespace + '/follow_joint_trajectory',
                                                          FollowJointTrajectoryAction,
                                                          execute_cb=self.process_follow_trajectory,
                                                          auto_start=False)
        self.action_server.start()
        Thread(target=self.update_state).start()

    def stop(self):
        self.running = False

    def process_command(self, msg):
        if self.action_server.is_active(): self.action_server.set_preempted()

        while self.action_server.is_active():
            rospy.sleep(0.01)

        self.process_trajectory(msg)

    def process_follow_trajectory(self, goal):
        self.process_trajectory(goal.trajectory)

    def process_trajectory(self, traj):
        num_points = len(traj.points)

        cur_pos = []
        for i,state in enumerate(self.motor_states):
            cur_pos.append( ( i+1, SvenzvaDriver.rad_to_raw(self.motor_states[i].position )))

        self.mx_io.set_multi_position(tuple(cur_pos))


        # make sure the joints in the goal match the joints of the controller
        if set(self.joint_names) != set(traj.joint_names):
            res = FollowJointTrajectoryResult()
            res.error_code=FollowJointTrajectoryResult.INVALID_JOINTS
            msg = 'Incoming trajectory joints do not match the joints of the controller'
            rospy.logerr(msg)
            rospy.logerr(' self.joint_names={}' % (set(self.joint_names)))
            rospy.logerr(' traj.joint_names={}' % (set(traj.joint_names)))
            self.action_server.set_aborted(result=res, text=msg)
            return

        # make sure trajectory is not empty
        if num_points == 0:
            msg = 'Incoming trajectory is empty'
            rospy.logerr(msg)
            self.action_server.set_aborted(text=msg)
            return

        # correlate the joints we're commanding to the joints in the message
        # map from an index of joint in the controller to an index in the trajectory
        lookup = [traj.joint_names.index(joint) for joint in self.joint_names]
        durations = [0.0] * num_points

        # find out the duration of each segment in the trajectory
        durations[0] = traj.points[0].time_from_start.to_sec()

        for i in range(1, num_points):
            durations[i] = (traj.points[i].time_from_start - traj.points[i - 1].time_from_start).to_sec()

        if not traj.points[0].positions:
            res = FollowJointTrajectoryResult()
            res.error_code=FollowJointTrajectoryResult.INVALID_GOAL
            msg = 'First point of trajectory has no positions'
            rospy.logerr(msg)
            self.action_server.set_aborted(result=res, text=msg)
            return

        trajectory = []
        time = rospy.Time.now() + rospy.Duration(0.01)

        for i in range(num_points):
            seg = Segment(self.num_joints)

            if traj.header.stamp == rospy.Time(0.0):
                seg.start_time = (time + traj.points[i].time_from_start).to_sec() - durations[i]
            else:
                seg.start_time = (traj.header.stamp + traj.points[i].time_from_start).to_sec() - durations[i]

            seg.duration = durations[i]

            # Checks that the incoming segment has the right number of elements.
            if traj.points[i].velocities and len(traj.points[i].velocities) != self.num_joints:
                res = FollowJointTrajectoryResult()
                res.error_code=FollowJointTrajectoryResult.INVALID_GOAL
                msg = 'Command point %d has %d elements for the velocities' % (i, len(traj.points[i].velocities))
                rospy.logerr(msg)
                self.action_server.set_aborted(result=res, text=msg)
                return

            if len(traj.points[i].positions) != self.num_joints:
                res = FollowJointTrajectoryResult()
                res.error_code=FollowJointTrajectoryResult.INVALID_GOAL
                msg = 'Command point %d has %d elements for the positions' % (i, len(traj.points[i].positions))
                rospy.logerr(msg)
                self.action_server.set_aborted(result=res, text=msg)
                return

            for j in range(self.num_joints):
                if traj.points[i].velocities:
                    seg.velocities[j] = traj.points[i].velocities[lookup[j]]
                if traj.points[i].positions:
                    seg.positions[j] = traj.points[i].positions[lookup[j]]

            trajectory.append(seg)

        rospy.loginfo('Trajectory start requested at %.3lf, waiting...', traj.header.stamp.to_sec())
        rate = rospy.Rate(self.update_rate)
        while traj.header.stamp > time:
            time = rospy.Time.now()
            rate.sleep()

        end_time = traj.header.stamp + rospy.Duration(sum(durations))
        seg_end_times = [rospy.Time.from_sec(trajectory[seg].start_time + durations[seg]) for seg in range(len(trajectory))]

        rospy.loginfo('Trajectory start time is %.3lf, end time is %.3lf, total duration is %.3lf', time.to_sec(), end_time.to_sec(), sum(durations))

        self.trajectory = trajectory
        traj_start_time = rospy.Time.now()

        for seg in range(len(trajectory)):
            rospy.logdebug('current segment is %d time left %f cur time %f' % (seg, durations[seg] - (time.to_sec() - trajectory[seg].start_time), time.to_sec()))
            rospy.logdebug('goal positions are: %s' % str(trajectory[seg].positions))

            # first point in trajectories calculated by OMPL is current position with duration of 0 seconds, skip it
            if durations[seg] == 0:
                rospy.logdebug('skipping segment %d with duration of 0 seconds' % seg)
                continue

            vals = []

            for joint in self.joint_names:
                j = self.joint_names.index(joint)

                start_position = self.motor_states[j].position
                if seg != 0: start_position = trajectory[seg - 1].positions[j]

                desired_position = trajectory[seg].positions[j]
                desired_velocity = max(self.min_velocity, abs(desired_position - start_position) / durations[seg])

                self.msg.desired.positions[j] = desired_position
                self.msg.desired.velocities[j] = desired_velocity

                motor_id = self.joint_to_id[joint]

                pos = SvenzvaDriver.rad_to_raw(desired_position * self.gear_ratios[j])
                spd = SvenzvaDriver.spd_rad_to_raw(desired_velocity * self.gear_ratios[j])
                vals.append((motor_id, pos, spd))
            self.mx_io.set_multi_position_and_speed(vals)

            while time < seg_end_times[seg]:
                # check if new trajectory was received, if so abort current trajectory execution
                # by setting the goal to the current position
                if self.action_server.is_preempt_requested():
                    msg = 'New trajectory received. Aborting old trajectory.'
                    multi_packet = {}

                    for j, joint in enumerate(self.joint_names):
                        cur_pos = self.motor_states[joint].position

                        motor_id = self.joint_to_id[joint]
                        pos = SvenzvaDriver.rad_to_raw(cur_pos)

                        vals.append((motor_id, pos, 0))

                    self.mx_io.set_multi_position_and_speed(vals)
                    self.action_server.set_preempted(text=msg)
                    rospy.logwarn(msg)
                    return

                rate.sleep()
                time = rospy.Time.now()

            # Verifies trajectory constraints
            for j, joint in enumerate(self.joint_names):
                if self.trajectory_constraints[j] > 0 and self.msg.error.positions[j] > self.trajectory_constraints[j]:
                    res = FollowJointTrajectoryResult()
                    res.error_code=FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
                    msg = 'Unsatisfied position constraint for %s, trajectory point %d, %f is larger than %f' % \
                           (joint, seg, self.msg.error.positions[j], self.trajectory_constraints[j])
                    rospy.logwarn(msg)
                    self.action_server.set_aborted(result=res, text=msg)
                    return

        # let motors roll for specified amount of time
        rospy.sleep(self.goal_time_constraint)

        for i, j in enumerate(self.joint_names):
            rospy.logdebug('desired pos was %f, actual pos is %f, error is %f' % (trajectory[-1].positions[i], self.motor_states[i].position, self.motor_states[i].position - trajectory[-1].positions[i]))

        # Checks that we have ended inside the goal constraints
        for (joint, pos_error, pos_constraint) in zip(self.joint_names, self.msg.error.positions, self.goal_constraints):
            if pos_constraint > 0 and abs(pos_error) > pos_constraint:
                res = FollowJointTrajectoryResult()
                res.error_code=FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
                msg = 'Aborting because %s joint wound up outside the goal constraints, %f is larger than %f' % \
                      (joint, pos_error, pos_constraint)
                rospy.logwarn(msg)
                self.action_server.set_aborted(result=res, text=msg)
                break
        else:
	    msg = 'Trajectory execution successfully completed'
	    rospy.loginfo(msg)
	    res = FollowJointTrajectoryResult()
	    res.error_code=FollowJointTrajectoryResult.SUCCESSFUL
            self.action_server.set_succeeded(result=res, text=msg)

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            self.msg.header.stamp = rospy.Time.now()

            # Publish current joint state
            for i, joint in enumerate(self.joint_names):
                state = self.motor_states[i]
                self.msg.actual.positions[i] = state.position
                self.msg.actual.velocities[i] = abs(state.speed)
                self.msg.error.positions[i] = self.msg.actual.positions[i] - self.msg.desired.positions[i]
                self.msg.error.velocities[i] = self.msg.actual.velocities[i] - self.msg.desired.velocities[i]

            self.state_pub.publish(self.msg)
            rate.sleep()
