#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


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
};

/*
 * Default mappings are for Xbox 360 gamepad
 * TODO: there need to be soft limits on the allowable velocity
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
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("joystick/cmd_vel", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &SvenzvaArmJoystick::joyCallback, this);

}

void SvenzvaArmJoystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  //twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_x];
  twist.linear.y = l_scale_*joy->axes[linear_y];
  twist.linear.z = l_scale_*joy->axes[linear_z];
  vel_pub_.publish(twist);
  r.sleep();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "svenzva_joystick_controller");
  SvenzvaArmJoystick svenzva_joy;

  ros::spin();
}
