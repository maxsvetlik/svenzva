This package permits the interpretation of joystick commands to linear and angular commands.
Current mappings supported: Xbox 360 controller

In order to use this package, one must have `xboxdrv` installed on your system.
    $ sudo apt-get install --install-recommends jstest* joystick xboxdrv

Next, the joystick must have the proper permissions set.
List the permissions with: 
    $ ls -l /dev/input/jsX
where X is an integer that describes the joystick of interest.

The permission output will look similar to:
    crw-rw-XX-

if XX is '--', the permissions need to be set to rw with:
    $ sudo chmod a+rw /dev/input/jsX


For full details, see the instructions in [this][http://wiki.ros.org/joy/Tutorials/WritingTeleopNode] tutorial.
