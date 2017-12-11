/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, University of Colorado, Boulder
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Univ of CO, Boulder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Original Author: Dave Coleman (https://github.com/davetcoleman/ros_control_boilerplate) */
/* Edited by: Dorian Goepp (https://github.com/resibots/dynamixel_control_hw/) */
/* Edited by: Max Svetlik (https://github.com/svenzvarobotics/) */
#include <sstream>

#include <dynamixel_control_hw/dynamixel_loop.hpp>
#include <dynamixel_control_hw/dynamixel_hardware_interface.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamixel_control_hw");
    ros::NodeHandle nh;

    // Get parameters for the hardware
    // -------------------------------

    ros::NodeHandle nhParams("~");
    std::string sub_namespace = nhParams.getNamespace();

    // Run the hardware interface node
    // -------------------------------

    // We run the ROS loop in a separate thread as external calls, such
    // as service callbacks loading controllers, can block the (main) control loop
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Create the hardware interface specific to your robot
    boost::shared_ptr<dynamixel::DynamixelHardwareInterface>
        dynamixel_hw_interface = boost::make_shared<dynamixel::DynamixelHardwareInterface>();
    dynamixel_hw_interface->init(nh);

    // Start the control loop
    dynamixel::DynamixelLoop control_loop(nh, dynamixel_hw_interface);

    // Wait until shutdown signal recieved
    ros::waitForShutdown();

    return 0;
}
