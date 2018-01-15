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

"""
Author: Maxwell Svetlik. Code snippets used and modified where indicated.
"""

import rospy
import rospkg
import actionlib
import yaml
import rospkg

from collections import defaultdict, deque
from threading import Thread
from mx_driver import dynamixel_io
from mx_driver.dynamixel_const import *
from svenzva_drivers.joint_trajectory_action_controller import *
from svenzva_drivers.revel_cartesian_controller import *
from svenzva_drivers.revel_arm_services import *
from svenzva_drivers.revel_gripper_server import *
from svenzva_drivers.svenzva_compliance_controller import *
from std_msgs.msg import Bool
from dynamixel_controllers.srv import *
from svenzva_drivers.msg import *
from svenzva_drivers.srv import *
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from svenzva_msgs.msg import MotorState, MotorStateList



class SvenzvaDriver:

    #adapted from controller_manager.py [https://github.com/SvenzvaRobotics/mx_dynamixel], 3/17/17
    def __init__(self,
                 port_name='/dev/ttyUSB0',
                 port_namespace='revel',
                 baud_rate='115200',
                 min_motor_id=1,
                 max_motor_id=7,
                 update_rate=10,
                 diagnostics_rate=0,
                 readback_echo=False):

        rospy.init_node('svenzva_driver', anonymous=False)

        self.port_name = port_name
        self.port_namespace = port_namespace
        self.baud_rate = baud_rate
        self.min_motor_id = min_motor_id
        self.max_motor_id = max_motor_id
        self.update_rate = rospy.get_param('~update_rate', update_rate)
        self.diagnostics_rate = diagnostics_rate
        self.readback_echo = readback_echo

        self.actual_rate = update_rate
        self.error_counts = {'non_fatal': 0, 'checksum': 0, 'dropped': 0}
        self.current_state = MotorStateList()
        self.num_ping_retries = 5

        self.motor_states_pub = rospy.Publisher('%s/motor_states' % self.port_namespace, MotorStateList,         queue_size=1)
        rospy.on_shutdown(self.disconnect)

        self.connect(port_name, baud_rate, False)
        self.__find_motors()
        self.initialze_motor_states()

        self.start_modules()

    #adapted from serial_proxy.py [https://github.com/SvenzvaRobotics/mx_dynamixel], 3/17/17
    def connect(self, port_name, baud_rate, readback_echo):
        try:
            self.dxl_io = dynamixel_io.DynamixelIO(port_name, baud_rate, readback_echo)
        except dynamixel_io.SerialOpenError, e:
            rospy.logfatal(e.message)
            sys.exit(1)

        if self.update_rate > 0: Thread(target=self.__update_motor_states).start()

    def disconnect(self):
        return

    """
    Check if all motors are reachable on the serial port
    """
    #adapted from serial_proxy.py [https://github.com/SvenzvaRobotics/mx_dynamixel], 3/17/17
    def __find_motors(self):
        rospy.loginfo('%s: Pinging motor IDs %d through %d...' % (self.port_namespace, self.min_motor_id, self.  max_motor_id))
        self.motors = []
        self.motor_static_info = {}

        for motor_id in range(self.min_motor_id, self.max_motor_id + 1):
            for trial in range(self.num_ping_retries):
                try:
                    result = self.dxl_io.ping(motor_id)
                except Exception as ex:
                    rospy.logerr('Exception thrown while pinging motor %d - %s' % (motor_id, ex))
                    continue
                if result:
                    self.motors.append(motor_id)
                    break
        if not self.motors:
            rospy.logfatal('%s: No motors found.' % self.port_namespace)
            self.dxl_io.close()
            sys.exit(1)

        counts = defaultdict(int)

        status_str = '%s: Found %d motors - ' % (self.port_namespace, len(self.motors))
        rospy.loginfo('%s, actuator initialization complete.' % status_str[:-2])


    #adapted from serial_proxy.py [https://github.com/SvenzvaRobotics/mx_dynamixel], 3/17/17
    def __update_motor_states(self):
        num_events = 50
        debug_polling_rate = False
        rates = deque([float(self.update_rate)]*num_events, maxlen=num_events)
        last_time = rospy.Time.now()
        gr = [4,6,6,1,4,1,1]
        rate = rospy.Rate(self.update_rate)
        id_list = range(self.min_motor_id, self.max_motor_id+1)
        rad_per_tick = 6.2831853 / 4096.0
        conseq_drops = 0

        while not rospy.is_shutdown():
            motor_states = []

            try:
                status_ar = self.dxl_io.get_sync_feedback(id_list)
                conseq_drops = 0
                for index, state in enumerate(status_ar):
                    if state:
                        #convert to radians, and resolve multiplicative of gear ratio
                        state['goal'] = self.raw_to_rad(state['goal']  / gr[index])
                        state['position'] = self.raw_to_rad(state['position'] / gr[index])
                        #linear model: -9.539325804e-18 + 1.0837745x
                        state['load'] = (state['load'] )
                        state['speed'] = self.spd_raw_to_rad(state['speed'] / gr[index])
                        motor_states.append(MotorState(**state))
                        if dynamixel_io.exception: raise dynamixel_io.exception
                self.error_counts['dropped'] = 0
            except dynamixel_io.FatalErrorCodeError, fece:
                rospy.logerr(fece)
            except dynamixel_io.NonfatalErrorCodeError, nfece:
                self.error_counts['non_fatal'] += 1
                rospy.logdebug(nfece)
            except dynamixel_io.ChecksumError, cse:
                self.error_counts['checksum'] += 1
                rospy.logdebug(cse)
            except dynamixel_io.DroppedPacketError, dpe:
                self.error_counts['dropped'] += 1
                conseq_drops += 1
                rospy.loginfo(dpe.message)
            except OSError, ose:
                if ose.errno != errno.EAGAIN:
                    rospy.logfatal(errno.errorcode[ose.errno])
                    rospy.signal_shutdown(errno.errorcode[ose.errno])

            #DroppedPackets can happen due to congestion, or due to loss of connectivity.
            #The latter will cause 100% drop rate
            if self.error_counts['dropped'] > 10:
                rospy.logerr("Lost connectivitity to servo motors.")
                rospy.logerr("Shutting down driver.")
                rospy.signal_shutdown("Arm connection lost.")

            if motor_states:
                msl = MotorStateList()
                msl.motor_states = motor_states
                self.motor_states_pub.publish(msl)

                self.current_state = msl

                # calculate actual update rate
                if debug_polling_rate:
                    current_time = rospy.Time.now()
                    rates.append(1.0 / (current_time - last_time).to_sec())
                    self.actual_rate = round(sum(rates)/num_events, 2)
                    last_time = current_time
                    rospy.loginfo("Actual poling rate: %f", self.actual_rate)
            rate.sleep()

    """
    This enables velocity control mode.
    Necessary for cartesian movement for remote controls
    """
    def velocity_mode(self):
        tup_list_dis = tuple(((1,0),(2,0),(3,0),(4,0),(5,0),(6,0),(7,0)))
        self.dxl_io.sync_set_torque_enabled(tup_list_dis)


        tup_list_op = tuple(((1,1),(2,1),(3,1),(4,1),(5,1),(6,1),(7,0)))
        self.dxl_io.sync_set_operation_mode(tup_list_op)

        tup_list_en = tuple(((1,1),(2,1),(3,1),(4,1),(5,1),(6,1),(7,1)))
        self.dxl_io.sync_set_torque_enabled(tup_list_en)



    """
    This enables the teaching mode of the Revel. Teaching mode senses outside forces and assists movement in the direction
    of the felt force.

    """
    def teaching_mode(self):

        tup_list_dis = tuple(((1,0),(2,0),(3,0),(4,0),(5,0),(6,0),(7,0)))
        self.dxl_io.sync_set_torque_enabled(tup_list_dis)


        tup_list_op = tuple(((1,0),(2,0),(3,0),(4,0),(5,0),(6,0),(7,0)))
        self.dxl_io.sync_set_operation_mode(tup_list_op)

        tup_list_en = tuple(((1,1),(2,1),(3,1),(4,1),(5,1),(6,1),(7,1)))
        self.dxl_io.sync_set_torque_enabled(tup_list_en)

        self.compliance_controller = SvenzvaComplianceController(self.port_namespace, self.dxl_io, True)
        rospy.sleep(0.1)
        Thread(target=self.compliance_controller.start).start()

    """
    Sets motor mode based on parameter file
    """
    def set_user_defined_mode(self, params):
        tup_list_dis = tuple(((1,0),(2,0),(3,0),(4,0),(5,0),(6,0),(7,0)))
        self.dxl_io.sync_set_torque_enabled(tup_list_dis)

        tup_list_op = []
        for i in range(self.min_motor_id, self.max_motor_id + 1):
            tup_list_op.append((i, params[i]["mode"]))
        self.dxl_io.sync_set_operation_mode(tup_list_op)

        tup_list_en = tuple(((1,1),(2,1),(3,1),(4,1),(5,1),(6,1),(7,1)))
        self.dxl_io.sync_set_torque_enabled(tup_list_en)



    def start_modules(self):
        global traj_client

        #compliance_demonstration is an experimental dynamic compliance module
        compliance_demonstration = rospy.get_param('~dynamic_compliance', False)

        jtac = JointTrajectoryActionController(self.port_namespace, self.dxl_io, self.current_state)
        rospy.sleep(1.0)
        jtac.start()

        self.fkine_action = actionlib.SimpleActionServer("svenzva_joint_action", SvenzvaJointAction, self.fkine_action, auto_start = False)
        self.fkine_action.start()

        arm_utils = RevelArmServices(self.port_namespace, self.dxl_io, self.max_motor_id)

        gripper_server = RevelGripperActionServer(self.port_namespace, self.dxl_io)
        gripper_server.start()

        cart_server = RevelCartesianController(self.port_namespace, self.dxl_io)

        rospy.loginfo("Started Cartesian controller")

        traj_client = actionlib.SimpleActionClient('/revel/follow_joint_trajectory', FollowJointTrajectoryAction)
        traj_client.wait_for_server()

        if compliance_demonstration:
            rospy.loginfo("Starting with experimental dynamic compliance.")
            self.compliance_controller = SvenzvaComplianceController(self.port_namespace, self.dxl_io,False)
            rospy.sleep(0.1)
            Thread(target=self.compliance_controller.start).start()
            #Thread(target=self.compliance_controller.update_state).start()

    """
    Initialize internal motor parameters that are reset when powered down.
    Enables torque mode.

    Uses settings in ../config/control_params.yaml
    """
    #NOTE: Due to dynamixel limitations, initial encoder values must be [-4096, 4096]
    #otherwise, the motor_states will be inaccurate
    def initialze_motor_states(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('svenzva_drivers')
        config_file = rospy.get_param('~param_file', 'control_params.yaml')

        params = ''
        with open( path+"/config/"+config_file, 'r') as stream:
            try:
                params = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                rospy.logerr("Unable to open control_params.yaml. Exiting driver.")
                exit()

        mode = rospy.get_param('~mode', "user_defined")
        teaching_mode = mode == "gravity"
        vel_mode = mode == "velocity"

        if teaching_mode:
            self.teaching_mode()
            return
        elif vel_mode:
            self.velocity_mode()
        else:
            #for nearly atomic context switch, use sync functions
            self.set_user_defined_mode(params)
            for i in range(self.min_motor_id, self.max_motor_id + 1):
                self.dxl_io.set_position_p_gain(i, params[i]['p'])
                self.dxl_io.set_position_i_gain(i, params[i]['i'])
                self.dxl_io.set_position_d_gain(i, params[i]['d'])
                self.dxl_io.set_acceleration_profile(i, params[i]['acceleration'])
                self.dxl_io.set_velocity_profile(i, params[i]['velocity'])


        #set current / torque limit for gripper
        self.dxl_io.set_goal_current(7, 0)
        self.dxl_io.set_current_limit(7, 1900)



    """
    TODO

    To increase reliability of packet transmission and reduce the number of packets required to fetch
    motor status, set the indirect addresses on each motor.
    This is REQUIRED to be called before starting any status based callbacks.
    """
    """
    def set_indirect_address(self):
        bulk write ( INDIR_ADDR_1, (1, MX_PRESENT_CURRENT), (2, MX_PRESENT_CURRENT), ... )
        bulk write ( INDIR_ADDR_1 + 2, (1, MX_PRESENT_CURRENT+1), (2, MX_PRESENT_CURRENT+1), ...)
        ...

    """



    """
    Given an array of joint positions (in radians), send request to individual servos
    """
    def fkine_action(self, data):
        global traj_client
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        point = JointTrajectoryPoint()
        point.positions = data.positions
        #The duration and waiting for goal affect smoothness of joint actions
        point.time_from_start = rospy.Duration(5.0)
        goal.trajectory.points.append(point)
        traj_client.send_goal_and_wait(goal)
        res = SvenzvaJointResult()
        res.is_done = True
        self.fkine_action.set_succeeded(res)

    @staticmethod
    def rad_to_raw(angle):
        #encoder ticks = resolution / radian range
        return int(round( angle * 4096.0 / 6.2831853 ))

    @staticmethod
    def raw_to_rad(raw):
        #radians_per = radians_range / resolution
        return raw * 6.2831853 / 4096.0

    @staticmethod
    def spd_rad_to_raw(vel):
        return max(1, int(round(vel / (RPM_TO_RADSEC * RPM_PER_TICK))))

    @staticmethod
    def spd_raw_to_rad(vel):
        return vel * RPM_PER_TICK * RPM_TO_RADSEC

if __name__ == '__main__':
    try:
        sd = SvenzvaDriver()
        rospy.spin()
        sd.disconnect()
    except rospy.ROSInterruptException:
        pass

