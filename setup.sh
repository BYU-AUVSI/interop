#!/bin/bash

# this script turns the interop repository into a catkin workspace
# it also downloads other needed packages
source /opt/ros/kinetic/setup.bash
apt-get update && apt-get install -y python-pip
pip install --upgrade pip
pip install requests
mkdir src
git clone --depth=50 https://github.com/BYU-AUVSI/vision_pkg.git src/vision_pkg
git clone -b RC1.0 --depth=50 https://github.com/BYU-AUVSI/fcu_common.git src/fcu_common
mkdir src/auvsi_client
SCRIPT_NAME=`basename "$0"`
for file in `ls -a | grep -xv '.' | grep -xv '..' | grep -v src | grep -v $SCRIPT_NAME | grep -v client.sh`; do
  mv $file src/auvsi_client/
done
rm -rf src/*.git*
catkin_init_workspace
rm CMakeLists.txt
catkin_make
