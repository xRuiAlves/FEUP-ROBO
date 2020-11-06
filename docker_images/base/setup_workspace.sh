#!/usr/bin/env sh

mkdir -p /root/catkin_ws/src
cd /root/catkin_ws

. /opt/ros/noetic/setup.sh

# catkin_make creates the workspace with all the necessary files
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
