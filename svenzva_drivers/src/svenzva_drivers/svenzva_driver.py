#!/usr/bin/env python

"""
Author: Maxwell Svetlik
"""

import rospy
import rospkg
import actionlib
import yaml

from collections import defaultdict, deque
from threading import Thread
from mx_driver import dynamixel_io
from mx_driver.dynamixel_const import *
from svenzva_drivers.joint_trajectory_action_controller import *
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

    #adapted from controller_manager.py [LINK], 3/17/17
    def __init__(self,
                 port_name='/dev/ttyUSB0',
                 port_namespace='revel',
                 baud_rate='1000000',
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

    #adapted from serial_proxy.py [LINK], 3/17/17
    def connect(self, port_name, baud_rate, readback_echo):
        try:
            self.dxl_io = dynamixel_io.DynamixelIO(port_name, baud_rate, readback_echo)
        except dynamixel_io.SerialOpenError, e:
            rospy.logfatal(e.message)
            sys.exit(1)

        if self.update_rate > 0: Thread(target=self.__update_motor_states).start()
        #if self.diagnostics_rate > 0: Thread(target=self.__publish_diagnostic_information).start()

    def disconnect(self):
        return
        #self.dxl_io.close()

    #Check if all motors are reachable on the serial port
    #adapted from serial_proxy.py [LINK], 3/17/17
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


    #adapted from serial_proxy.py [LINK], 3/17/17
    def __update_motor_states(self):
        num_events = 50
        debug_polling_rate = False
        rates = deque([float(self.update_rate)]*num_events, maxlen=num_events)
        last_time = rospy.Time.now()
        gr = [4,6,6,4,4,1,1]
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
                        #convert raw current to torque model (in newton meters)
                        #linear model: -9.539325804e-18 + 1.0837745x
                        state['load'] = (state['load'] ) #* .00336 ) * 1.083775 - 9.54e-18#1.14871 - .1244557
                        state['speed'] = self.spd_raw_to_rad(state['speed'] / gr[index])
                        motor_states.append(MotorState(**state))
                        if dynamixel_io.exception: raise dynamixel_io.exception
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
                rospy.shutdown()

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


    def compliance_example_j4(self):
        self.dxl_io.set_torque_enabled(1, 1)
        self.dxl_io.set_torque_enabled(2, 1)
        self.dxl_io.set_torque_enabled(3, 1)

        self.dxl_io.set_operation_mode(1, 0) #change back to 5 for pos
        self.dxl_io.set_operation_mode(2, 0) #change back to 5 for pos
        self.dxl_io.set_operation_mode(3, 0) #change back to 5 for pos


        self.dxl_io.set_operation_mode(4, 0) #change back to 5 for pos
        self.dxl_io.set_torque_enabled(4, 1)


        self.dxl_io.set_operation_mode(4, 0) #change back to 5 for pos
        self.dxl_io.set_torque_enabled(5, 1)

        self.dxl_io.set_operation_mode(6, 0) #change back to 5 for pos
        self.dxl_io.set_torque_enabled(6, 1)


        #TODO: make standalone
        compliance_controller = SvenzvaComplianceController(self.port_namespace, self.dxl_io)
        compliance_controller.start(0.02)

        #below are good for compliance
        #self.dxl_io.set_acceleration_profile(4, 40)
        #self.dxl_io.set_velocity_profile(4, 200)


    def start_modules(self):
        jtac = JointTrajectoryActionController(self.port_namespace, self.dxl_io, self.current_state)
        rospy.sleep(1.0)
        jtac.start()

        action = actionlib.SimpleActionServer("svenzva_joint_action", SvenzvaJointAction, self.fkine_action, auto_start = False)
        action.start()

        arm_utils = RevelArmServices(self.port_namespace, self.dxl_io, self.max_motor_id)

        gripper_server = RevelGripperActionServer(self.port_namespace, self.dxl_io)
        gripper_server.start()

        """
        Svenzva.SvenzvaPoseActionServer pose_server(comm, nh, kinova_robotType);
        Svenzva.SvenzvaAnglesActionServer angles_server(comm, nh);
        Svenzva.SvenzvaFingersActionServer fingers_server(comm, nh);
        """


    #TODO: read from yaml
    """
    Initialize internal motor parameters that are reset when powered down.
    Enables torque mode.
    """
    #NOTE: Due to dynamixel limitations, initial encoder values must be [-4096, 4096]
    #otherwise, the motor_states will be inaccurate
    def initialze_motor_states(self):
        #self.dxl_io.set_torque_enabled(5, 1)
        #self.dxl_io.set_goal_current(5, 0)
        compliance_for_j4 = True

        if compliance_for_j4:
            self.compliance_example_j4()
            return
        else:
            self.dxl_io.set_operation_mode(4, 5) #change back to 5 for pos

        for i in range(self.min_motor_id, self.max_motor_id + 1):
            self.dxl_io.set_torque_enabled(i, 1)
            #self.dxl_io.set_operation_mode(i, 4)
            #self.dxl_io.set_position_p_gain(i, 2048)
            #self.dxl_io.set_position_i_gain(i, 0)

            #below are good for compliance
            #self.dxl_io.set_acceleration_profile(i, 40)
            #self.dxl_io.set_velocity_profile(i, 200)

            #below are good for trajectories
            self.dxl_io.set_acceleration_profile(i, 5)
            self.dxl_io.set_velocity_profile(i, 80)
            rospy.sleep(0.1)
    """
    Given an array of joint positions (in radians), send request to individual servos
    TODO: Check if enought joint positions
          Check if motors are in joint mode and not wheel mode
    """
    def fkine_action(self, data):
        traj_client = actionlib.SimpleActionClient('/revel/follow_joint_trajectory', FollowJointTrajectoryAction)
        traj_client.wait_for_server()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        point = JointTrajectoryPoint()
        point.positions = data.positions
        point.time_from_start = rospy.Duration(5.0)
        goal.trajectory.points.append(point)
        traj_client.send_goal_and_wait(goal)


    @staticmethod
    def rad_to_raw(angle):
        #return int(rouna(angle * encoder_ticks_per_rad))
        #encoder ticks = resolution / radian range
        return int(round( angle * 4096.0 / 6.2831853 ))

    @staticmethod
    def raw_to_rad(raw):
        #return raw * radians_per_encoder_tick
        #radians_per = radians_range / resolution
        return raw * 6.2831853 / 4096.0

    @staticmethod
    def spd_rad_to_raw(vel):
        return max(1, int(round(vel / (RPM_TO_RADSEC * RPM_PER_TICK))))

    @staticmethod
    def spd_raw_to_rad(vel):
        return vel * RPM_PER_TICK * RPM_TO_RADSEC

def home_arm(data):
    # load the yaml file that specifies the home position
    rospack = rospkg.RosPack()
    path = rospack.get_path('svenzva_drivers')
    f = open(path+"/config/home_position.yaml")
    qmap = yaml.safe_load(f)
    f.close()

    #2 - execute action on yaml file
    req = SvenzvaJointGoal()
    if len(qmap['home']) < 6:
        rospy.logerr("Could not home arm. Home position configuration file ill-formed or missing. Aborting.")
        return
    req.positions = qmap['home']

    fkine = actionlib.SimpleActionClient('/svenzva_joint_action', SvenzvaJointAction)

    fkine.wait_for_server()
    rospy.loginfo("Found Trajectory action server")
    rospy.loginfo("Homing arm...")
    fkine.send_goal_and_wait(req)

def setup():
    rospy.init_node('svenzva_driver', anonymous=True)
    action = actionlib.SimpleActionServer("svenzva_joint_action", SvenzvaJointAction, fkine_action, auto_start = False)
    action.start()
    rospy.Service('home_arm_service', HomeArm, home_arm)
    while not rospy.is_shutdown():
        rospy.spin()
if __name__ == '__main__':
    try:
        #setup()
        sd = SvenzvaDriver()
        rospy.spin()
        sd.disconnect()
    except rospy.ROSInterruptException:
        pass

