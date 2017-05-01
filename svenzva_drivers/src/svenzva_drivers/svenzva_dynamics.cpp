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

#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <sensor_msgs/JointState.h>

sensor_msgs::JointState joint_states;
KDL::Tree my_tree;
KDL::Chain chain;
int mNumJnts = 6;
// Get some joint pos, vel, acc values
KDL::JntArray jnt_q(mNumJnts);
KDL::JntArray jnt_qd(mNumJnts);
KDL::JntArray jnt_qdd(mNumJnts);
KDL::JntArray jnt_taugc(mNumJnts);

void js_cb(const sensor_msgs::JointState::ConstPtr& msg){
    joint_states = *msg;
}

/*
 * Inverse velocity kinematics
 */

void feel_velocity(){
    
    for (unsigned int i = 0; i < mNumJnts; i++) {
      jnt_q(i) = 0.0;
      jnt_qd(i) = 0.0;
      jnt_qdd(i) = 0.0;
    }
    jnt_q(1) = 0.5;
    jnt_q(2) = 0.72;
    KDL::ChainIkSolverVel_pinv vel_solver = KDL::ChainIkSolverVel_pinv(chain,0.00001,150);

    //Find an output joint velocity qdot_out, given a starting joint pose q_init and a desired cartesian velocity v_in 
    KDL::Vector trans(0.01, 0.0, 0.0);
    KDL::Vector rot(0, 0, 0);
    KDL::Twist vel(trans, rot);
    KDL::JntArray qdot_out(mNumJnts);
    vel_solver.CartToJnt(  jnt_q, vel, qdot_out);  
    ROS_INFO("Here3");

    for( int i=0; i < mNumJnts; i++){
        ROS_INFO("Joint %d: %f", i+1, qdot_out(i));
    }

}


/*
 * tau given q and the system dynamics
 */
void feel_efforts(ros::Publisher tau_pub){
    KDL::Wrenches jnt_wrenches;
    for (unsigned int i = 0; i < mNumJnts; i++) {
      jnt_q(i) = joint_states.position[i];
      jnt_qd(i) = 0.0;
      jnt_qdd(i) = 0.0;
      jnt_wrenches.push_back(KDL::Wrench());
    }

    // Kinematics 
    KDL::ChainFkSolverPos_recursive fkSolver = KDL::ChainFkSolverPos_recursive(chain);
    KDL::Frame fkKDL;
    fkSolver.JntToCart(jnt_q, fkKDL);

    // Compute Dynamics 
    KDL::Vector gravity(0.0, 0.0, -9.81);
    KDL::ChainIdSolver_RNE gcSolver = KDL::ChainIdSolver_RNE(chain, gravity);
    int ret = gcSolver.CartToJnt(jnt_q, jnt_qd, jnt_qdd, jnt_wrenches,jnt_taugc);
    sensor_msgs::JointState model_states;
    model_states = joint_states;
    if (ret < 0){ 
        ROS_ERROR("KDL: inverse dynamics ERROR");
        ROS_ERROR("%d", ret);
    }
    else{
        int divisor = 1;
        for( int i = 0; i < mNumJnts; i++){
            //ROS_INFO("Joint %d got %f", i+1, jnt_taugc(i));
            //Compute the error in the model vs present output torques
            if (i == 1 || i == 2)
                divisor = 6;
            else if (i == 3 || i == 4)
                divisor = 4;
            model_states.effort[i] = jnt_taugc(i) / divisor;
            //double error = joint_states.effort[i] - (jnt_taugc(i) / divisor);
            //ROS_INFO("Joint %d error: %f. %f theory vs %f actual ", i+1, error, (jnt_taugc(i) / divisor), joint_states.effort[i]); 
        }
    }
    tau_pub.publish(model_states);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "svenzva_dynamics");

    ros::NodeHandle n;
    int rate = 10;
    //n.param<int>("~publish_rate", rate, 20);
    ros::Subscriber js_sub = n.subscribe("joint_states", 2, js_cb);
    ros::Publisher tau_pub = n.advertise<sensor_msgs::JointState>("/revel/model_efforts/", 1);
    ros::Rate update_rate = ros::Rate(rate);
    std::string path = ros::package::getPath("svenzva_description");
    std::string full_path = path + "/robots/svenzva_arm.urdf";
    ROS_INFO("Loading model from %s", full_path.c_str());
    
    kdl_parser::treeFromFile(full_path, my_tree);
    
    if (!kdl_parser::treeFromFile(full_path, my_tree)){
       ROS_ERROR("Failed to construct kdl tree");
       return false;
    }
    my_tree.getChain("base_link", "link_6", chain);
    ROS_INFO("Kinematic chain expects %d joints", chain.getNrOfJoints());


    /*
     * Publish the expected torque according to the dynamic model
     */
    ros::spinOnce();
    ros::Rate(1).sleep();
    update_rate.sleep();
    while(ros::ok()){
        ros::spinOnce();
        feel_efforts(tau_pub);
        update_rate.sleep();
    }
    
    /*
     * Testing out jacobian and differential kinematics
     */
    //feel_velocity();

    return 0;

}
