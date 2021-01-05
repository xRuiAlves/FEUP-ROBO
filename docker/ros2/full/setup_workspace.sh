#!/usr/bin/env sh

mkdir -p /root/robo/src
cd /root/robo

. /opt/ros/foxy/setup.sh

# Dockerfile puts this in /root/robo
vcs import src < turtlebot3.repos

colcon build --symlink-install
