#!/usr/bin/env python

import rospy
import pprint

from dynamic_reconfigure.server import Server
from svenzva_drivers.cfg import RevelFirmwareDynamicConfig


class RevelDynamicParameterServer():

    def __init__(self, controller_namespace, mx_io):
        self.is_first_configuration = True
        self.last_configuration = {}

    def callback(self, config, level):
        #first run
        if self.is_first_configuration:
            self.last_configuration = config.copy()
            self.is_first_configuration = False
            return config

        #if user hasn't enabled parameter control, do not process parameter changes
        if not config.enable_parameter_control:
            return self.last_configuration

        #process parameter changes
        if level & 1: #acceleration profile
            if config.joint_1_acc != self.last_configuration.joint_1_acc:
                self.set_acceleration_profile(1, config.joint_1_vel)
            elif config.joint_2_acc != self.last_configuration.joint_2_acc:
                self.set_accleration_profile(2, config.joint_2_vel)
            elif config.joint_3_acc != self.last_configuration.joint_3_acc:
                self.set_acceleration_profile(3, config.joint_3_vel)
            elif config.joint_4_acc != self.last_configuration.joint_4_acc:
                self.set_acceleration_profile(4, config.joint_4_vel)
            elif config.joint_5_acc != self.last_configuration.joint_5_acc:
                self.set_acceleration_profile(5, config.joint_5_vel)
            elif config.joint_6_acc != self.last_configuration.joint_6_acc:
                self.set_acceleration_profile(6, config.joint_6_vel)
            elif config.joint_7_acc != self.last_configuration.joint_7_acc:
                self.set_acceleration_profile(7, config.joint_7_acc)

        if level & 1<<1: #velocity profile
            if config.joint_1_vel != self.last_configuration.joint_1_vel:
                self.set_velocity_profile(1, config.joint_1_vel)
            elif config.joint_2_vel != self.last_configuration.joint_2_vel:
                self.set_velocity_profile(2, config.joint_2_vel)
            elif config.joint_3_vel != self.last_configuration.joint_3_vel:
                self.set_velocity_profile(3, config.joint_3_vel)
            elif config.joint_4_vel != self.last_configuration.joint_4_vel:
                self.set_velocity_profile(4, config.joint_4_vel)
            elif config.joint_5_vel != self.last_configuration.joint_5_vel:
                self.set_velocity_profile(5, config.joint_5_vel)
            elif config.joint_6_vel != self.last_configuration.joint_6_vel:
                self.set_velocity_profile(6, config.joint_6_vel)
            elif config.joint_7_vel != self.last_configuration.joint_7_vel:
                self.set_velocity_profile(7, config.joint_7_vel)

        if level & 1<<2: # Joint 1 PID
            self.set_position_PID(1, config.joint_1_P, config.joint_1_I, config.joint_1_D)
        if level & 1<<3: #Joint 2 PID
            self.set_position_PID(1, config.joint_2_P, config.joint_2_I, config.joint_2_D)
        if level & 1<<4: #Joint 3 PID
            self.set_position_PID(1, config.joint_3_P, config.joint_3_I, config.joint_3_D)
        if level & 1<<5: #Joint 4 PID
            self.set_position_PID(1, config.joint_4_P, config.joint_4_I, config.joint_4_D)
        if level & 1<<6: #Joint 5 PID
            self.set_position_PID(1, config.joint_5_P, config.joint_5_I, config.joint_5_D)
        if level & 1<<7: #Joint 6 PID
            self.set_position_PID(1, config.joint_6_P, config.joint_6_I, config.joint_6_D)
        if level & 1<<8: #Joint 7 PID
            self.set_position_PID(1, config.joint_7_P, config.joint_7_I, config.joint_7_D)

        self.last_configuration = config.copy()
        return config

    def set_position_PID(self, motor_id, p_gain_val, i_gain_val, d_gain_val, ff1_gain, ff2_gain):
        self.mx_io.set_position_p_gain(self, servo_id, p_gain_val)
        self.mx_io.set_position_i_gain(self, servo_id, i_gain_val)
        self.mx_io.set_position_d_gain(self, servo_id, d_gain_val)
        self.mx_io.set_position_feedfwd1_gain(self, servio_id, ff1_gain)
        self.mx_io.set_position_feedfwd2_gain(self, servio_id, ff2_gain)

    def set_acceleration_profile(self, motor_id, gain):
        self.mx_io.set_acceleration_profile(motor_id, gain)

    def set_velocity_profile(self, motor_id, gain):
        self.mx_io.set_velocity_profile(motor_id, gain)

    def start(self):
        srv = Server(RevelFirmwareDynamicConfig, self.callback)
        rospy.spin()

if __name__ == "__main__":
        rospy.init_node("svenzva_drivers_reconfigure", anonymous = False)
        rdps = RevelDynamicParameterServer('', '')
        rdps.start()
