#!/usr/bin/env sh

cd /root/catkin_ws

. /opt/ros/noetic/setup.sh

. devel/setup.sh

cd src
catkin_create_pkg robo std_msgs rospy roscpp turtlebot3_gazebo

git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ..

catkin_make
