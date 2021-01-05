#!/usr/bin/env sh
rocker --oyr-run-arg " --name robo-rocker -v $(pwd)/../../source/:/root/robo/src/robo_fi/" --devices /dev/dri/card0 --x11 ros_fault_injection:full
