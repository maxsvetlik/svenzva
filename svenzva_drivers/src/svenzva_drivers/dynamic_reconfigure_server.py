#!/usr/bin/env python

import rospy
import pprint

from dynamic_reconfigure.server import Server
from svenzva_drivers.cfg import RevelFirmwareDynamicConfig


def callback(config, level):
    global is_first_configuration, last_configuration
    #pp = pprint.PrettyPrinter(indent=4)
    #pp.pprint(config.groups.groups)
    #config.groups.groups.acceleration.state = False

    if is_first_configuration:
        config.groups.groups.acceleration.state = False
        config.groups.groups.velocity.state = False
        config.groups.groups.pid.state = False
        last_configuration = config.copy()
        is_first_configuration = False
        return config

    if not config.enable_parameter_control:
        return last_configuration

    for g in config.groups.groups:
        if config.groups.groups[g].state == False:
            config.groups.groups[g] = last_configuration.groups.groups[g]

    return config

if __name__ == "__main__":
    global is_first_configuration, last_configuration
    is_first_configuration = True
    last_configuration = {}

    rospy.init_node("svenzva_drivers_reconfigure", anonymous = False)

    srv = Server(RevelFirmwareDynamicConfig, callback)

    rospy.spin()
