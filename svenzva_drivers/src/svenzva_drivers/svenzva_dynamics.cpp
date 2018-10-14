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
//  * Neither the name of Svenzva Robotics LLC nor the names of its
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
#include <cmath>

#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <sensor_msgs/JointState.h>


/*
 * SvenzvaDynamics is a ROS component that is used in Kinesthetic Teaching of the Revel robot.
 *
 * Given the current robot state, this class uses a dynamic model to estimate the gravity load 
 * on each joint due to the robot's own physical characteristics.
 * 
 * This class has many opportunities for fine-tuning parameters if the force estimation is inaccurate.
 */

sensor_msgs::JointState joint_states;
KDL::Tree my_tree;
KDL::Chain chain;
int mNumJnts = 6;
// Get some joint pos, vel, acc values
KDL::JntArray jnt_q(mNumJnts);
KDL::JntArray jnt_qd(mNumJnts);
KDL::JntArray jnt_qdd(mNumJnts);
KDL::JntArray jnt_taugc(mNumJnts);
sensor_msgs::JointState model_states;
bool first_run = true;
KDL::ChainIdSolver_RNE *gcSolver;

double alpha = 0.5;

void js_cb(const sensor_msgs::JointState::ConstPtr& msg){
    for(int i = 0; i < msg->effort.size(); i++){
        joint_states.effort[i] = msg->effort[i] * alpha + (1.0 - alpha) * joint_states.effort[i];
    }
    joint_states.position = msg->position;
    //joint_states = *msg;
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
      KDL::Wrench wr = KDL::Wrench();
      if( !first_run) {
        int gr = 1;
        double diff = model_states.effort[i] - joint_states.effort[i];
        double frc = 0.0;

        frc = -1 * (diff) * gr;
        
        KDL::Vector force(0, frc, 0);
        wr.torque = force;
        jnt_wrenches.push_back(wr);
      }
      else{
        jnt_wrenches.push_back(KDL::Wrench());
      }
    }

    first_run = false;
    // Compute Dynamics 
    int ret = gcSolver->CartToJnt(jnt_q, jnt_qd, jnt_qdd, jnt_wrenches,jnt_taugc);
    model_states = joint_states;
    if (ret < 0){ 
        ROS_ERROR("KDL: inverse dynamics ERROR");
        ROS_ERROR("%d", ret);
    }
    else{
        int divisor = 1;
        for( int i = 0; i < mNumJnts; i++){
            //Compute the error in the model vs present output torques
            if( i == 1){
                //spring function: y= -0.000012159725010x^3 + 0.001933370127203x^2 + 0.002502918014389x + 0.075320089110718
                double spring_offset = (-0.0001215972501*std::pow(joint_states.position[i], 3)) + (0.0019337012*std::pow(joint_states.position[i], 2)) + (0.0025029181439*joint_states.position[i]) + 0.0753200891107  ;
                jnt_taugc(i) = jnt_taugc(i) - spring_offset; 
            }
            
            if (i == 0)
                divisor = 10000;
            if (i == 1 || i == 2)
                divisor = 7;
            else if (i == 3)
                divisor = 3;
            else if (i == 4)
                divisor = 4;
            model_states.effort[i] = jnt_taugc(i) / divisor;
        }
    }
    tau_pub.publish(model_states);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "svenzva_dynamics");

    ros::NodeHandle n;
    if (n.getParam("numberMotors", mNumJnts))
    {
      ROS_INFO("Got param!!!");
    }
    else
    {
      ROS_ERROR("Failed to get param 'my_param'");
    }

    for(int i = 0; i < mNumJnts+1; i++)
        joint_states.effort.push_back(0.0);

    ros::Subscriber js_sub = n.subscribe("joint_states", 1, js_cb);
    ros::Publisher tau_pub = n.advertise<sensor_msgs::JointState>("/revel/model_efforts/", 1);
    ros::Rate update_rate = ros::Rate(10);
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

    KDL::Vector gravity(0.0, 0.0, -9.81);
    gcSolver = new KDL::ChainIdSolver_RNE(chain, gravity);


    /*
     * Publish the expected torque according to the dynamic model
     */
    ros::spinOnce();
    ros::Duration(5).sleep();
    update_rate.sleep();
    while(ros::ok()){
        ros::spinOnce();
        feel_efforts(tau_pub);
        update_rate.sleep();
    }
    
    return 0;

}
