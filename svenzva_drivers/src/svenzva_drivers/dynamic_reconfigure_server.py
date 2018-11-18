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
        if level & 1:
            print "Changed acceleration profile!"
            print level
        if level & 1<<1:
            print "Changed velocity profile!"
            print level
        if level & 1<<2:
            print "Changed Joint 1 PID!"
            print level
        if level & 1<<3:
            print "Changed Joint 2 PID!"
            print level
        if level & 1<<4:
            print "Changed Joint 3 PID!"
            print level
        if level & 1<<5:
            print "Changed Joint 4 PID!"
            print level
        if level & 1<<6:
            print "Changed Joint 5 PID!"
            print level
        if level & 1<<7:
            print "Changed Joint 6 PID!"
            print level
        if level & 1<<8:
            print "Changed Joint 7 PID!"
            print level
        return config

    #def set_position_p_gain(self, servo_id, p_gain):
    #def set_position_PID(self, ):
        #mx_io.set_position_p_gain(self, servo_id, p_gain):

    def start(self):
        srv = Server(RevelFirmwareDynamicConfig, self.callback)
        rospy.spin()

if __name__ == "__main__":
        rospy.init_node("svenzva_drivers_reconfigure", anonymous = False)
        rdps = RevelDynamicParameterServer('', '')
        rdps.start()
