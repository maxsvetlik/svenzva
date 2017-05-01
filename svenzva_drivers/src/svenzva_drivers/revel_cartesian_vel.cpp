// Software License Agreement (BSD License)
//
// Copyright (c) 2017 Svenzva Robotics
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of University of Arizona nor the names of its
//   contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>

#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <sensor_msgs/JointState.h>
#include <svenzva_msgs/SvenzvaJointAction.h>
#include <geometry_msgs/Twist.h>

sensor_msgs::JointState joint_states;
KDL::Tree my_tree;
KDL::Chain chain;
int mNumJnts = 6;
// Get some joint pos, vel, acc values
KDL::JntArray jnt_q(mNumJnts);
KDL::JntArray jnt_qd(mNumJnts);
KDL::JntArray jnt_qdd(mNumJnts);
KDL::JntArray jnt_taugc(mNumJnts);
svenzva_msgs::SvenzvaJointGoal goal;
bool vel_updated = false;


void js_cb(const sensor_msgs::JointState::ConstPtr& msg){
    joint_states = *msg;
}

/*
 * Inverse velocity kinematics
 */

void cart_vel_cb(const geometry_msgs::TwistConstPtr& msg){
    if(msg->linear.x == 0 && msg->linear.y == 0 && msg->linear.z == 0)
        return;

    for (unsigned int i = 0; i < mNumJnts; i++) {
      jnt_q(i) = joint_states.position[i];
      jnt_qd(i) = 0.0;
      jnt_qdd(i) = 0.0;
    }
    
    KDL::ChainIkSolverVel_pinv vel_solver = KDL::ChainIkSolverVel_pinv(chain,0.00001,150);

    //Find an output joint velocity qdot_out, given a starting joint pose q_init and a desired cartesian velocity v_in 
    KDL::Vector trans(msg->linear.x, msg->linear.y, -1 * msg->linear.z);
    KDL::Vector rot(msg->angular.x, msg->angular.y, -1 * msg->angular.z);
    KDL::Twist vel(trans, rot);
    KDL::JntArray qdot_out(mNumJnts);

    //Rather than using joint velocity (which requires a context switch in motor EEPROM)
    //get joint positions that satisfy the trans and rotation given.
    //TODO: switch to velocity, and extrapolate position so as to send position trajectories on a constant, but incremental basis
    //so as to behave as one would expect a velocity command to
    vel_solver.CartToJnt(  jnt_q, vel, qdot_out);
    
    svenzva_msgs::SvenzvaJointGoal goal_prime;
    ros::spinOnce();

    for( int i=0; i < mNumJnts; i++){
        //ROS_INFO("Joint %d: %f", i+1, qdot_out(i));
        ROS_INFO("Joint at %f, moving to %f", joint_states.position[i], joint_states.position[i] + qdot_out(i));
        goal_prime.positions.push_back(joint_states.position[i] + qdot_out(i));
    }
    goal = goal_prime;
    vel_updated = true;

    if(false){
        //send position trajectory to driver
        //j_ac.sendGoal(goal);
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "revel_cartesian_vel_manager");

    ros::NodeHandle n;
    int rate = 10;
    //n.param<int>("~publish_rate", rate, 20);
    ros::Subscriber sub = n.subscribe("/revel/eef_velocity", 1, cart_vel_cb);
    ros::Subscriber js_sub = n.subscribe("joint_states", 2, js_cb);
    ros::Rate update_rate = ros::Rate(rate);
    std::string path = ros::package::getPath("svenzva_description");
    std::string full_path = path + "/robots/svenzva_arm.urdf";

    actionlib::SimpleActionClient<svenzva_msgs::SvenzvaJointAction> j_ac("svenzva_joint_action", true);

    //setup kinematic model
    ROS_INFO("Loading model from %s", full_path.c_str());

    kdl_parser::treeFromFile(full_path, my_tree);

    if (!kdl_parser::treeFromFile(full_path, my_tree)){
       ROS_ERROR("Failed to construct kdl tree");
       return false;
    }
    my_tree.getChain("base_link", "link_6", chain);
    ROS_INFO("Kinematic chain expects %d joints", chain.getNrOfJoints());

    ROS_INFO("Waiting for svenzva_joint_action server to start.");
    j_ac.waitForServer();


    while(ros::ok()){
        ros::spinOnce();
        //hacky use of private action server?
        if(vel_updated)
            j_ac.sendGoal(goal);
        vel_updated = false;
        update_rate.sleep();
    }

    return 0;

}

