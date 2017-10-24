#include <dynamixel_control_hw/dynamixel_hardware_interface.hpp>

#include <stdexcept>
#include <limits>

#include <math.h>

/* Written by: Dorian Goepp (https://github.com/resibots/dynamixel_control_hw/) */
/* Edited by: Max Svetlik (https://github.com/svenzvarobotics/) */

namespace dynamixel {
    DynamixelHardwareInterface::DynamixelHardwareInterface() 
    {}
    
    DynamixelHardwareInterface::~DynamixelHardwareInterface()
    {
    }

    void DynamixelHardwareInterface::init(ros::NodeHandle nh)
    {
        _prev_commands.resize(6, 0.0);
        _joint_commands.resize(6, 0.0);
        _joint_angles.resize(6, 0.0);
        _joint_velocities.resize(6, 0.0);
        _joint_efforts.resize(6, 0.0);
        try {
            
            for (unsigned i = 0; i < 6; i++) {
                
                std::string j_name = "joint_" + std::to_string(i+1);
                // tell ros_control the in-memory address where to read the
                // information on joint angle, velocity and effort
                hardware_interface::JointStateHandle state_handle(
                    j_name,
                    &_joint_angles[i],
                    &_joint_velocities[i],
                    &_joint_efforts[i]);
                _jnt_state_interface.registerHandle(state_handle);
                // tell ros_control the in-memory address to change to set new
                // position goal for the actuator
                hardware_interface::JointHandle pos_handle(
                    _jnt_state_interface.getHandle(j_name),
                    &_joint_commands[i]);
                _jnt_effort_interface.registerHandle(pos_handle);
                
            }
            
            // register the hardware interfaces
            registerInterface(&_jnt_state_interface);
            registerInterface(&_jnt_effort_interface);
            
        }
        catch (const ros::Exception& e) {
            ROS_ERROR_STREAM("Could not initialize hardware interface:\n\tTrace: " << e.what());
            throw e;
        }

        // At startup robot should keep the pose it has
        ros::spinOnce();

        for (unsigned i = 0; i < 6; i++) {
            _joint_commands[i] = _joint_angles[i];
        }
    }

    /** Copy joint's information to memory

        firstly queries the information from the dynamixels, then put it in private
        attributes, for use by a controller.

        Warning: do not get any information on torque
    **/
    void DynamixelHardwareInterface::read_joints(sensor_msgs::JointState js)
    {
        for (unsigned i=0; i < 6; i++){
            _joint_angles[i] = js.position[i];
            _joint_efforts[i] = js.effort[i];
            _joint_velocities[i] = js.velocity[i];
        }
        
    }

    /** Send new joint's target position to dynamixels

        takes the target position from memory (given by a controller) and sends
        them to the dynamixels.
    **/
    std::vector<double> DynamixelHardwareInterface::write_joints()
    {
        _prev_commands = _joint_commands;
        return _joint_commands;
    }
}
