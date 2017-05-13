#!/bin/bash

source ~/dubcar/dubcar_ws/devel/setup.bash
export ROS_MASTER_URI=http://Ubuntu16:11311
exec "$@"
