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
        self.chain = self.tree.getChain('base_link', 'ee_link')
        #print chain.getNrOfJoints()
        self.mNumJnts = 6
        self.jnt_q = PyKDL.JntArray(self.mNumJnts);
        self.jnt_qd = PyKDL.JntArray(self.mNumJnts);
        self.jnt_qdd = PyKDL.JntArray(self.mNumJnts);
        self.gear_ratios = [4,6,6,4,4,1]
        self.js = JointState()
        self.epsilon = 0.001
        self.last_twist = Twist()
        self.last_cmd = []
        self.last_qdot = PyKDL.JntArray(self.mNumJnts)

    def js_cb(self, msg):
        self.js = msg;

    def rad_to_raw(self, angle):
        return int(round( angle * 4096.0 / 6.2831853) )

    def cart_vel_cb(self, msg):
        rospy.Rate(4).sleep()
        """
        if msg == self.last_twist:
            if msg.linear.x == 0 and msg.linear.y == 0 and msg.linear.z == 0:
                return
            tup_list = []
            for i in range(0, self.mNumJnts):
                tup_list.append( (i+1, self.rad_to_raw( (self.js.position[i]+self.last_qdot[i]) * self.gear_ratios[i])) )
            self.mx_io.set_multi_position(tuple(tup_list))
            return
        else:
            self.last_twist = msg
        """
        for i in range(0, self.mNumJnts):
            self.jnt_q[i] = self.js.position[i];
            self.jnt_qd[i] = 0.0;
            self.jnt_qdd[i] = 0.0;

        vel_solver = PyKDL.ChainIkSolverVel_pinv(self.chain, 0.00001, 150);
        trans = PyKDL.Vector(msg.linear.x, msg.linear.y, msg.linear.z)
        qdot_out = PyKDL.JntArray(self.mNumJnts)
        vel = PyKDL.Twist(trans, PyKDL.Vector(0, 0, 0))

        err = vel_solver.CartToJnt(self.jnt_q, vel, qdot_out)
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
        for i in range(0, self.mNumJnts):
            if abs(qdot_out[i]) > 0.5:
                #compute scale factor that would make movement valid:
                my_scale = abs(0.5 / qdot_out[i])
                if my_scale < scale_factor:
                    scale_factor = my_scale
            #vals.append( (self.js.position[i] + qdot_out[i]) * self.gear_ratios[i])

            if not self.robot.joints[i].limit.lower <= (qdot_out[i]*scale_factor) + self.js.position[i] <= self.robot.joints[i].limit.upper:
                rospy.logwarn("Cartesian movement would cause movement outside of joint limits. Skipping...")
                return
            #    rospy.logdebug("qdot value: %f, current pos: %f", qdot_out[i], self.js.position[i])
            #    #compute scale factor that would make movement valid:
            #if abs(qdot_out[i]) > 0.5:
            #    rospy.logerr("qdot value: %f, current pos: %f", qdot_out[i], self.js.position[i])
            #    rospy.logerr("Velocity too large for joint %d. Skipping motion.", i+1)
            #    return
            #    rospy.loginfo("Joint at %f, moving to %f", self.js.position[i], self.js.position[i] + qdot_out[i])

        if scale_factor < 1:
            rospy.loginfo("Scaling all velocity by %f", scale_factor)

        for i in range(0, self.mNumJnts):
            if abs( scale_factor * qdot_out[i]) > self.epsilon:

                tup_list.append( (i+1, self.rad_to_raw( (self.js.position[i] + (qdot_out[i] * scale_factor)) * self.gear_ratios[i])))


        if len(tup_list) > 0:
            self.last_cmd = tup_list
            self.last_qdot = qdot_out
            self.mx_io.set_multi_position(tuple(tup_list))

