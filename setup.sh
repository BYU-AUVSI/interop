#!/bin/bash

source /opt/ros/kinetic/setup.bash
apt-get update && apt-get install -y python-pip \
  ros-kinetic-gazebo-ros-control
pip install --upgrade pip
pip install requests
mkdir src
mkdir src/auvsi_client
SCRIPT_NAME=`basename "$0"`
for file in `ls -a | grep -xv '.' | grep -xv '..' | grep -v src | grep -v $SCRIPT_NAME | grep -v client.sh`; do
  mv $file src/auvsi_client/
done
git clone --depth=50 https://github.com/BYU-AUVSI/fcu_common.git src/fcu_common
rm -rf src/fcu_common/.git*
catkin_init_workspace
rm CMakeLists.txt
catkin_make
