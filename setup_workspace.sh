#!/bin/bash

# REQUIRED
# -Supported Ubuntu distro
# -Python 2/3

# DIRECTIONS
# Execute from root directory

echo "MUST RUN 'source ~/.bashrc AFTER!"
echo 
echo

function USAGE {
  echo "Usage: $0 [username of current, non-root user (i.e \$USER)]"
  echo "Don't forget to run 'source ~/.bashrc' after"
  exit
}

if [ "$#" -ne 1 ]; then
  USAGE
fi

# configure Ubuntu to look for packages in packages.ros.org
add-apt-repository "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main restricted universe multiverse"
add-apt-repository "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc)-updates main restricted universe multiverse"
add-apt-repository ppa:roehling/latest

# setup keys for connecting to ros package server
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# update package index
apt-get update

# install ros base
apt-get install ros-kinetic-ros-base

# initialize rosdep for handling system dependencies
rosdep init
# DO NOTE EXECUTE FOLLOWING COMMAND AS ROOT
sudo -u $1 rosdep update

# install wstools (better than rosinstall)
apt-get install python-wstool

# install catkin_tools (DONT USE CATKIN)
apt-get install python-catkin-tools

# for checking good conventions in CMakelists/package.xml, and ros style
apt-get install python-catkin-lint
apt-get install ros-kinetic-roslint

# install rqt_graph for visualization
apt-get install ros-kinetic-rqt-graph

# install all dependencies
cd dubcar_ws
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

# setup ros environment paths
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# set ros param server to enable stats (required for rqt_graph)
rosparam set enable_statistics true
