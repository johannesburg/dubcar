#!/bin/bash

# this file is listed in /etc/rc.local to be executed on boot as user root


export ROS_MASTER_URI=http://Ubuntu16:11311 
source DUBCAR_ROOT/dubcar_ws/devel/setup.bash 
#sudo /opt/ros/kinetic/lib/ps3joy/ps3joy.py --continuous-output --inactivity-timeout=300
roslaunch dubcar dubcar.launch
exit 0
