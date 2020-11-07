#!/usr/bin/env sh
rocker --oyr-run-arg " -v $(pwd)/../robo_src/:/root/catkin_ws/src/robo/" --devices /dev/dri/card0 --x11 robo:turtled 
# rocker --devices /dev/dri/card0 --x11 robo:turtled 
