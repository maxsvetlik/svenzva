#include <dynamixel_control_hw/dynamixel_hardware_interface.hpp>

#include <stdexcept>
#include <limits>

#include <math.h>

/* Written by: Dorian Goepp (https://github.com/resibots/dynamixel_control_hw/) */

namespace dynamixel {
    DynamixelHardwareInterface::DynamixelHardwareInterface(const std::string& usb_serial_interface,
        const int& baudrate,
        const float& read_timeout,
        const float& scan_timeout,
        std::map<long long int, std::string> dynamixel_map,
        std::map<long long int, long long int> dynamixel_max_speed,
        std::map<long long int, double> dynamixel_corrections)
        : _usb_serial_interface(usb_serial_interface),
          _baudrate(get_baudrate(baudrate)),
          _read_timeout(read_timeout),
          _scan_timeout(scan_timeout),
          _dynamixel_map(dynamixel_map),
          _dynamixel_max_speed(dynamixel_max_speed),
          _dynamixel_corrections(dynamixel_corrections)
    {
    }

    DynamixelHardwareInterface::~DynamixelHardwareInterface()
    {
        /*
        // stop all actuators
        try {
            for (auto dynamixel_servo : _dynamixel_servos) {
                dynamixel::StatusPacket<dynamixel::protocols::Protocol1> status;
                _dynamixel_controller.send(dynamixel_servo->set_torque_enable(0));
                _dynamixel_controller.recv(status);
            }
        }
        catch (dynamixel::errors::Error& e) {
            ROS_FATAL_STREAM("Caught a Dynamixel exception while trying to power them off:\n"
                << e.msg());
            throw e;
        }
        */
    }

    void DynamixelHardwareInterface::init(ros::NodeHandle nh)
    {
        _prev_commands.resize(1, 0.0);
        _joint_commands.resize(1, 0.0);
        _joint_angles.resize(1, 0.0);
        _joint_velocities.resize(1, 0.0);
        _joint_efforts.resize(1, 0.0);
        try {
            
            for (unsigned i = 0; i < 1; i++) {
                
                if (true) //(dynamixel_iterator != _dynamixel_map.end()) // check that the actuator's name is in the map
                {
                    // tell ros_control the in-memory address where to read the
                    // information on joint angle, velocity and effort
                    hardware_interface::JointStateHandle state_handle(
                        "joint_1",
                        &_joint_angles[i],
                        &_joint_velocities[i],
                        &_joint_efforts[i]);
                    _jnt_state_interface.registerHandle(state_handle);
                    // tell ros_control the in-memory address to change to set new
                    // position goal for the actuator
                    hardware_interface::JointHandle pos_handle(
                        _jnt_state_interface.getHandle("joint_1"),
                        &_joint_commands[i]);
                    _jnt_pos_interface.registerHandle(pos_handle);
                }
                else {
                    ROS_WARN_STREAM("Servo " << i << " was not initialised (not found in the parameters)");
                }
            }
            
            // register the hardware interfaces
            registerInterface(&_jnt_state_interface);
            registerInterface(&_jnt_pos_interface);
        
        }
        catch (const ros::Exception& e) {
            ROS_ERROR_STREAM("Could not initialize hardware interface:\n\tTrace: " << e.what());
            throw e;
        }

        // At startup robot should keep the pose it has
    }

    /** Copy joint's information to memory

        firstly queries the information from the dynamixels, then put it in private
        attributes, for use by a controller.

        Warning: do not get any information on torque
    **/
    void DynamixelHardwareInterface::read_joints(sensor_msgs::JointState js)
    {
        for (unsigned i=0; i < 1; i++){
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
        std_msgs::Float64 t_cmd;
        t_cmd.data = _joint_commands[0];
        //torque_pub.publish(t_cmd.data);       
        return _joint_commands;
    }
}
