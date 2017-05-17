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

class SvenzvaArmJoystick
{
public:
  SvenzvaArmJoystick();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_x, linear_y, linear_z, angular_, rate_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  ros::Rate r;
  sensor_msgs::Joy  last_cmd;
};

/*
 * Default mappings are for Xbox 360 gamepad
 */
SvenzvaArmJoystick::SvenzvaArmJoystick():
  linear_x(0),
  linear_y(1),
  linear_z(4),
  angular_(2),
  rate_(20),
  r(20)
{ 
  nh_.param("rate", rate_, rate_);
  nh_.param("axis_linear_x", linear_x, linear_x);
  nh_.param("axis_linear_y", linear_y, linear_y);
  nh_.param("axis_linear_z", linear_z, linear_z);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  r = ros::Rate(rate_);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("revel/eef_velocity", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &SvenzvaArmJoystick::joyCallback, this);

}

void SvenzvaArmJoystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  //twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_x];
  twist.linear.y = l_scale_*joy->axes[linear_y];
  twist.linear.z = l_scale_*joy->axes[linear_z];
  vel_pub_.publish(twist);
  last_cmd = *joy;


  if(joy->axes[2] < 0 || joy->axes[5] < 0){
      actionlib::SimpleActionClient<svenzva_msgs::GripperAction> gripper_action("/revel/gripper_action", true);
      gripper_action.waitForServer();
      
        svenzva_msgs::GripperGoal goal;
        if(joy->axes[2] < 0){
            goal.target_action = goal.CLOSE;
            goal.target_current = 100;
        }
        else if(joy->axes[5] < 0){
            goal.target_action = goal.OPEN;
        }

        gripper_action.sendGoal(goal);
  }


}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "svenzva_joystick_controller");
  SvenzvaArmJoystick svenzva_joy;
  actionlib::SimpleActionClient<svenzva_msgs::GripperAction> gripper_action("/revel/gripper_action", true);
  gripper_action.waitForServer();
  
  while(ros::ok()){
    /*
    svenzva_msgs::GripperGoal goal;
    if(last_cmd.axes[2] < 0){
        goal.target_action = goal.CLOSE;
        goal.target_current = 100;
    }
    else if(last_cmd.axes[5] < 0){
        goal.target_action = goal.OPEN;
    }

    gripper_action.sendGoal(goal);
    */
    ros::spinOnce();

  }
}
