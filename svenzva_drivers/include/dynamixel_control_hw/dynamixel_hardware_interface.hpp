#ifndef dynamixel_hardware_interface
#define dynamixel_hardware_interface

// ROS
#include <ros/ros.h>

// ROS control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

namespace dynamixel {

    class DynamixelHardwareInterface : public hardware_interface::RobotHW {
    public:
        DynamixelHardwareInterface();
        ~DynamixelHardwareInterface();

        void init(ros::NodeHandle nh);

        void read_joints(sensor_msgs::JointState js);
        std::vector<double> write_joints();

    private:
        // not implemented
        DynamixelHardwareInterface(DynamixelHardwareInterface const&);

        // not implemented
        DynamixelHardwareInterface& operator=(DynamixelHardwareInterface const&);

        // ROS's hardware interface instances
        hardware_interface::JointStateInterface _jnt_state_interface;
        hardware_interface::EffortJointInterface _jnt_effort_interface;

        // Memory space shared with the controller
        // It reads here the latest robot's state and put here the next desired values
        std::vector<std::string> _joint_names;
        std::vector<double> _prev_commands;
        std::vector<double> _joint_commands; // target joint angle
        std::vector<double> _joint_angles; // actual joint angle
        std::vector<double> _joint_velocities; // actual joint velocity
        std::vector<double> _joint_efforts; // compulsory but not used
        ros::Publisher torque_pub;
    };
}

#endif
