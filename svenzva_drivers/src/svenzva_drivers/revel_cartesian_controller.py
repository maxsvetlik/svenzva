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
A python version of the corresponding cpp functions of the same name.
This allows this to be run as part of the overall driver, and thus have
direct access to the communications layer.

Author Maxwell Svetlik
Copyright Svenzva Robotics

"""
import rospy
import rospkg
import PyKDL
import math

from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import Robot
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class RevelCartesianController:

    def __init__(self, controller_namespace, mx_io):
        rospy.Subscriber("/revel/eef_velocity", Twist, self.cart_vel_cb, queue_size=1);
        rospy.Subscriber("joint_states", JointState, self.js_cb, queue_size=1);
        rospack = rospkg.RosPack()
        path = rospack.get_path("svenzva_description");
        full_path = path + "/robots/svenzva_arm.urdf";

        f = file(full_path, 'r')
        self.robot = Robot.from_xml_string(f.read())
        f.close()

        self.mx_io = mx_io
        self.tree = kdl_tree_from_urdf_model(self.robot)
        self.chain = self.tree.getChain('base_link', 'link_6')
        self.mNumJnts = 6
        self.jnt_q = PyKDL.JntArray(self.mNumJnts);
        self.jnt_qd = PyKDL.JntArray(self.mNumJnts);
        self.jnt_qdd = PyKDL.JntArray(self.mNumJnts);
        self.gear_ratios = [4,6,6,3,4,1]
        self.js = JointState()
        self.min_limit = 20.0
        self.max_limit = 100.0
        self.last_twist = Twist()
        self.last_cmd = []
        self.last_qdot = PyKDL.JntArray(self.mNumJnts)
        self.vel_solver = PyKDL.ChainIkSolverVel_pinv(self.chain, 0.001, 1000);
        #self.vel_solver = PyKDL.ChainIkSolverVel_wdls(self.chain, 0.001, 1000000)
        self.cart_vel = Twist()
        self.arm_speed_limit = rospy.get_param('arm_speed_limit', 20.0)


        self.loop()

    def js_cb(self, msg):
        self.js = msg;

    def rad_to_raw(self, angle):
        return int(round( angle * 4096.0 / 6.2831853) )

    def radpm_to_rpm(self, rad_per_min):
        return rad_per_min * 0.159155

    def cart_vel_cb(self, msg):
        self.cart_vel = msg

    def loop(self):
        rospy.sleep(1.0)
        while not rospy.is_shutdown():
            msg = self.cart_vel

            #if msg == Twist():
            #    rospy.sleep(0.25)
            #    continue

            for i in range(0, self.mNumJnts):
                self.jnt_q[i] = self.js.position[i];
                self.jnt_qd[i] = 0.0;
                self.jnt_qdd[i] = 0.0;

            trans = PyKDL.Vector(msg.linear.x, msg.linear.y, msg.linear.z)
            rot = PyKDL.Vector(msg.angular.x, msg.angular.y, msg.angular.z)
            qdot_out = PyKDL.JntArray(self.mNumJnts)
            vel = PyKDL.Twist(trans, rot)
            err = self.vel_solver.CartToJnt(self.jnt_q, vel, qdot_out)
            if err == 1: #PyKDL.E_CONVERGE_PINV_SINGULAR:
                rospy.loginfo("Cartesian movement solver converged but gave degraded solution. Skipping.")
                return
            elif err == -8: #PyKDL.E_SVD_FAILED:
                rospy.loginfo("Cartesian movement solver did not converge. Skipping.")
                return
            elif err == 100:
                rospy.loginfo("Cartesian movement converged but psuedoinverse in singular.")
            elif err != 0:
                rospy.loginfo("Unspecified error: %d", err)
                return

            tup_list = []
            vals = []

            scale_factor = 1

            #check if overall arm velocity violates the ARM_SPEED_LIMIT
            #Note that the ARM_SPEED_LIMIT caps the arm output, where other speed parameters,
            #such as linear_scale and angular_scale in JOY nodes, caps the Twist input to this node.

            acc = 0.0
            for i in range(0, self.mNumJnts):
                acc += math.pow(qdot_out[i], 2)

            vel_scale = math.sqrt(acc) / self.arm_speed_limit
            if vel_scale > 1.0:
                scale_factor = vel_scale

            if scale_factor != 1:
                rospy.loginfo("Scaling all velocity by %f", 1/scale_factor)

            for i in range(0, self.mNumJnts-1):
                #check if movement violates urdf joint limits
                if self.robot.joints[i+1].limit.lower >= self.js.position[i] + (qdot_out[i]/scale_factor*0.01) or self.js.position[i] + (qdot_out[i]/scale_factor*0.01) >= self.robot.joints[i+1].limit.upper:
                    rospy.logwarn("Cartesian movement would cause movement outside of joint limits. Skipping...")
                    rospy.logwarn("Movement would violate joint limit: Joint %d moving to %f with limits [%f,%f]", i+1, (qdot_out[i]/scale_factor) + self.js.position[i], self.robot.joints[i+1].limit.lower, self.robot.joints[i+1].limit.upper)
                    tup_list.append( (i+1, 0))
                else:
                    tup_list.append( (i+1, int(round(self.radpm_to_rpm(qdot_out[i] * self.gear_ratios[i] / scale_factor) / 0.229 ))))

            if len(tup_list) > 0:
                self.last_cmd = tup_list
                self.last_qdot = qdot_out
                self.mx_io.set_multi_speed(tuple(tup_list))

            rospy.sleep(0.1)


