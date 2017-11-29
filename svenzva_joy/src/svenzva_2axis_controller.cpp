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

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <svenzva_msgs/GripperAction.h>
#include <actionlib/client/simple_action_client.h>

/*
 * This handles the joystick / velocity interface for the 
 * Svenzva Robotics custom 3 + 3 axis joystick controller. 
 *
 */

typedef actionlib::SimpleActionClient<svenzva_msgs::GripperAction> gripperClient;

class SvenzvaArmJoystick
{
public:
  SvenzvaArmJoystick();
  sensor_msgs::Joy  last_cmd;
  int gripper_button;

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, rate_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  ros::Rate r;
};

SvenzvaArmJoystick::SvenzvaArmJoystick():
  linear_x(0),
  linear_y(1),
  linear_z(4),
  angular_x(4),
  angular_y(4),
  angular_z(4),
  gripper_button(0),
  rate_(20),
  r(20)
{ 
  nh_.param("rate", rate_, rate_);
  nh_.param("axis_linear_x", linear_x, linear_x);
  nh_.param("axis_linear_y", linear_y, linear_y);
  nh_.param("axis_linear_z", linear_z, linear_z);
  nh_.param("axis_angular_x", angular_x, angular_x);
  nh_.param("axis_angular_y", angular_y, angular_y);
  nh_.param("axis_angular_z", angular_z, angular_z);
  nh_.param("gripper_button", gripper_button, gripper_button);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  r = ros::Rate(rate_);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("revel/eef_velocity", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &SvenzvaArmJoystick::joyCallback, this);
 
}

void SvenzvaArmJoystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;

  twist.linear.x = l_scale_*joy->axes[linear_x];
  twist.linear.y = l_scale_*joy->axes[linear_y];
  twist.linear.z = l_scale_*joy->axes[linear_z];
  twist.angular.x = a_scale_*joy->axes[angular_x];
  twist.angular.y = a_scale_*joy->axes[angular_y];
  twist.angular.z = a_scale_*joy->axes[angular_z];
  vel_pub_.publish(twist);
  last_cmd = *joy;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "svenzva_2axis_controller");
  SvenzvaArmJoystick svenzva_joy;
  
  gripperClient gripper_action("/revel/gripper_action", true);
  gripper_action.waitForServer();
  bool gripper_open = true; 

  ros::Duration(0.5).sleep();

  while(ros::ok()){
    ros::spinOnce();
    if(svenzva_joy.last_cmd.buttons[svenzva_joy.gripper_button] == 0){
     
      svenzva_msgs::GripperGoal goal;
      if(gripper_open){
          goal.target_action = goal.CLOSE;
          goal.target_current = 300;
          gripper_open = false;
      }
      else{
          goal.target_action = goal.OPEN;
          gripper_open = true;
      }
      gripper_action.sendGoalAndWait(goal);
    }
    ros::Rate(10).sleep();
  }


}
