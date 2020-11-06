FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y git nano vim && apt-get clean

COPY setup_workspace.sh /setup_workspace.sh
RUN /setup_workspace.sh

