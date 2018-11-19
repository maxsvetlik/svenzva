#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2018 Svenzva Robotics LLC
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
#  * Neither the name of Svenzva Robotics LLC nor the names of its
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
Dynamic parameter server for control loop parameters for the Revel robotic arm
and its motors.

Each motor can have its individual
    Positional P, I, D, feedforward_velocity, feedforward_acceleration
    velocity_profile (and speed)
    acceleration_profile (and acceleration)

where the velocity and acceleration profiles are determined by a comparison of values
between the two profiles. The values themselves set the target motor speed and acceleration
while the profile determines the motor speed and acceleration over time.

See Dynamixel 2.0 Firmware Manual, sections Profile Velocity and Profile Acceleration for more details.


NOTE: Setting control parameters manually can cause the control loop to become unstable,
potentially harming the environment and the robot.

To change control parameters, you must first check the 'enable control parameters' checkbox in the
rqt_reconfigure window. You then proceed AT YOUR OWN RISK.

Author: Maxwell Svetlik
"""


import rospy

from dynamic_reconfigure.server import Server
from svenzva_drivers.cfg import RevelFirmwareDynamicConfig


class RevelDynamicParameterServer():

    def __init__(self, controller_namespace, mx_io):
        self.is_first_configuration = True
        self.last_configuration = {}
        self.mx_io = mx_io
        self.controller_namespace = controller_namespace

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
                self.set_acceleration_profile(1, config.joint_1_acc)
            elif config.joint_2_acc != self.last_configuration.joint_2_acc:
                self.set_acceleration_profile(2, config.joint_2_acc)
            elif config.joint_3_acc != self.last_configuration.joint_3_acc:
                self.set_acceleration_profile(3, config.joint_3_acc)
            elif config.joint_4_acc != self.last_configuration.joint_4_acc:
                self.set_acceleration_profile(4, config.joint_4_acc)
            elif config.joint_5_acc != self.last_configuration.joint_5_acc:
                self.set_acceleration_profile(5, config.joint_5_acc)
            elif config.joint_6_acc != self.last_configuration.joint_6_acc:
                self.set_acceleration_profile(6, config.joint_6_acc)
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
            self.set_position_PID(1, config.joint_1_P, config.joint_1_I, config.joint_1_D, config.joint_1_Feedforward1_velocity, config.joint_1_Feedforward2_acceleration)
        if level & 1<<3: #Joint 2 PID
            self.set_position_PID(2, config.joint_2_P, config.joint_2_I, config.joint_2_D, config.joint_2_Feedforward1_velocity, config.joint_2_Feedforward2_acceleration)
        if level & 1<<4: #Joint 3 PID
            self.set_position_PID(3, config.joint_3_P, config.joint_3_I, config.joint_3_D, config.joint_3_Feedforward1_velocity, config.joint_3_Feedforward2_acceleration)
        if level & 1<<5: #Joint 4 PID
            self.set_position_PID(4, config.joint_4_P, config.joint_4_I, config.joint_4_D, config.joint_4_Feedforward1_velocity, config.joint_4_Feedforward2_acceleration)
        if level & 1<<6: #Joint 5 PID
            self.set_position_PID(5, config.joint_5_P, config.joint_5_I, config.joint_5_D, config.joint_5_Feedforward1_velocity, config.joint_5_Feedforward2_acceleration)
        if level & 1<<7: #Joint 6 PID
            self.set_position_PID(6, config.joint_6_P, config.joint_6_I, config.joint_6_D, config.joint_6_Feedforward1_velocity, config.joint_6_Feedforward2_acceleration)
        if level & 1<<8: #Joint 7 PID
            self.set_position_PID(7, config.joint_7_P, config.joint_7_I, config.joint_7_D, config.joint_7_Feedforward1_velocity, config.joint_7_Feedforward2_acceleration)

        self.last_configuration = config.copy()
        return config

    def set_position_PID(self, motor_id, p_gain_val, i_gain_val, d_gain_val, ff1_gain, ff2_gain):
        self.mx_io.set_position_p_gain(motor_id, p_gain_val)
        self.mx_io.set_position_i_gain(motor_id, i_gain_val)
        self.mx_io.set_position_d_gain(motor_id, d_gain_val)
        self.mx_io.set_position_feedfwd1_gain(motor_id, ff1_gain)
        self.mx_io.set_position_feedfwd2_gain(motor_id, ff2_gain)

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
