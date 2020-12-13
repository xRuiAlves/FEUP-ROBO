#!/usr/bin/env sh
# rocker --oyr-run-arg " -v $(pwd)/../robo_src/:/root/catkin_ws/src/robo/" --devices /dev/dri/card0 --x11 robo2:turtled 
# rocker --devices /dev/dri/card0 --x11 robo2:turtled 
rocker --oyr-run-arg " --name robo-rocker -v $(pwd)/../../robo2_src/:/root/robo/src/robo_fi/" --devices /dev/dri/card0 --x11 robo2:full
