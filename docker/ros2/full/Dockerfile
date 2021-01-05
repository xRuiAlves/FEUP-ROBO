FROM osrf/ros:foxy-desktop

RUN apt-get update && apt-get install -y git nano vim ros-foxy-gazebo-ros-pkgs && apt-get clean

WORKDIR /root/robo/
COPY turtlebot3.repos .
COPY setup_workspace.sh /setup_workspace.sh
RUN /setup_workspace.sh

ENV TURTLEBOT3_MODEL=burger
ENV GAZEBO_MODEL_PATH=/root/robo/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
