# Svenzva Robotics ROS drivers

This is the active repository for the Svenzva Robotics robotic product line ROS software.
The `svenzva_ros` package holds all thats needed to get up and running with ROS, including drivers, description    files, simulation files and interactive utilities.


### Prerequisites

Our testing platform is ROS Indigo with 14.04 and ROS Kinetic 16.04. You must have either of these installed or  have access to a system with ROS installed to use this ROS package.
You can find out more about installing ROS [here](http://wiki.ros.org/kinetic/Installation).

### Installing

These instructions will get you a copy of the ROS package and its dependencies on your local machine.

Clone the repository into an existing workspace
```
git clone https://github.com/SvenzvaRobotics/svenzva_ros.git
```
Use wstool to grab github source dependencies:
```
wstool init src PATH_TO_ROSINSTALL_FILE.rosinstall
```
where the rosinstall file desired is the __dependencies.rosinstall__ file in the root of the `svenzva_ros` package.

Finally, use rosdep to resolve dependencies specified in package descriptors:
```
rosdep install --from-paths ./ --ignore-src --rosdistro=$ROS_DISTRO -y
```


## Deployment

To bringup the ROS driver, bringup the main launch file:
```
roslaunch svenzva_drivers svenzva_bringup.launch
```

For additional detail on the bringing up the system, please check the [Github Wiki](https://github.com/SvenzvaRobotics/svenzva_ros/wiki).

## How to Use

Once the main launch file has been brought up, a number of robot features are exposed via ROS. 
We explore these in detail in the [Github Wiki](https://github.com/SvenzvaRobotics/svenzva_ros/wiki) pages and own support documents.

## License

This project, unless specified otherwise, is licensed under the BSD License - see the [LICENSE.md](LICENSE.md)   file for details.
