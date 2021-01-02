****************************************************************
***                                                          ***
***              ROS 2 Fault Injection Toolkit               ***
***                                                          ***
****************************************************************
***                                                          ***
***       Faculty of Engineering, University of Porto        ***
***           MIEIC MSc Robotics course 2020/2021            ***
***                                                          ***
***     Filipa Manita Santos Durão, up201606640@fe.up.pt     ***
***       Miguel Pereira Duarte, up201606298@fe.up.pt        ***
***  Rui Pedro Moutinho Moreira Alves, up201606746@fe.up.pt  ***
***                                                          ***
****************************************************************


Table of Contents:
- Versions and Dependencies
- Directory Organization
- Docker setup
- Creating a new Docker image (setting up a new package)
- Using rocker
- Setting up the 'hook' channel
- Running the Gazebo world
- Running a Turtlebot3 simulation
- Introducing an injectors pipeline


*************************************
***   Versions and Dependencies   ***
*************************************

- Docker v19.03.13-ce, build 4484c46d9d
- Rocker v0.2.2
- Python v3.8.2
- Ros 2 Foxy (ros:foxy-desktop docker image)


**********************************
***   Directory Organization   ***
**********************************

.
├── docker            # Docker configurations
│   └── ros2
│       ├── build.sh
│       ├── full      # Docker image
│       │   ├── Dockerfile
│       │   ├── setup_workspace.sh
│       │   └── turtlebot3.repos
│       └── rocker_run.sh
├── readme.txt        # This README file
└── source            # Injector scripts package source code
    ├── package.xml
    ├── resource
    │   └── robo_fi
    ├── robo_fi
    │   ├── base.py
    │   ├── communication.py
    │   ├── fault_injector.py
    │   ├── __init__.py
    │   ├── injection_types.py
    │   └── test_listen.py
    ├── setup.cfg
    └── setup.py





************************
***   Docker Setup   ***
************************

Access the docker directory:

$ cd docker

Build the docker image:

$ docker build -t ros_fault_injection:full full/ 


******************************************************
***   Creating a new Docker image (not required)   ***
******************************************************

This step is not mandatory; You may use one of the sample Turtlebot3 projects that are shipped with this toolkit (such as the Driving Simulator).

In order to make a new project's source code available, one needs to indicate the instruction to fetch that code, build it, and package it.

To do so, create a new Dockerfile that uses ros_fault_injection:full as the base image; Then, add instructions to:
1) Download the code;
2) Place it in a package in the root of the user;
3) Update the project's listening channel to a "hook channel", such as "scan_fi"
4) Compile the package


************************
***   Using Rocker   ***
************************

Given that docker containers are fully self-contained, one needs to use a tool to access windowed user interfaces (such as gazebo or rviz).

We recommend using rocker:

$ rocker --oyr-run-arg " --name robo-rocker -v $(pwd)/../../robo2_src/:/root/robo/src/robo_fi/" --devices /dev/dri/card0 --x11 ros_fault_injection:full

Alternatively, you may simply run the "rocker_run.sh" utility script:

$ ./rocker_run.sh

Either these commands will launch an interactive bash inside the container, which will have permissions to launch windowed UIs such as gazebo and rviz.
To make sure every ROS binary is accessible, run the following command (as explained in the docker ROS tutorial):

$ source install/setup.bash

Compile all the packages:

$ colcon build


***************************************
***   Setting up the hook channel   ***
***************************************

If section "Creating a new Docker image" was skipped, that means the Driving Simulator Turtlebot3 sample project will be used; Thus, one needs to setup the hook channel.

To do so, simply access the packages source code and update the listening channel from "scan" to, for example, "scan_fi" (in line 46):

$ nano src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/src/turtlebot3_drive.cpp

Then, make sure to re-compile the updated package:

$ colcon build


************************************
***   Running the Gazebo world   ***
************************************

To launch the gazebo GUI, run:

$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


*******************************************
***   Running a Turtlebot3 simulation   ***
*******************************************

Run, for example, the Turtlebot3 driving simulation:

$ ros2 run turtlebot3_gazebo turtlebot3_drive


*********************************************
***   Introducing an injectors pipeline   ***
*********************************************

The toolkit is available using the "fault_injector" script family from "robo_fi" package.

To check tool's usage:

$ ros2 run robo_fi fault_injector -h
> usage: fault_injector [-h] [-from FROM] [-to TO] -fi type [args ...]
>
> Fault Injection Toolkit for ROS2

> optional arguments:
>   -h, --help           show this help message and exit
>   -from FROM           topic to read from
>   -to TO               topic to write to
>   -fi type [args ...]  fault injection pipeline (can specify several injection types)

Both the "from" and "to" flags are optional and default to "scan" and "scan_fi", respectively.

The "fi" flags specifies a fault injection. The available injection are:

- fixed : sets LIDAR distance values to a fixed value
- addc : adds a constant to all LIDAR distance values
- null : sets all LIDAR distance values to null
- scale : scales all LIDAR distance values based on a given ratio
- random : sets all LIDAR distance values to random values between 0 and 1 (uniform distribution)
- rotate : rotates the LIDAR distance values readings array by a given number of positions

These fault injectors may be part of a complex pipeline. For example:

$ ros2 run robo_fi fault_injector \
$   -from scan
$   -to scan_fi
$   -fi rotate 10
$   -fi scale 2
$   -fi addc 1

This injection pipeline would feature 3 injectors:

- A LIDAR readings are read from the "scan" channel
- The first injector rotates the LIDAR distance values array by 10 positions
- The second injector scales the LIDAR distance values by a rate of 2 (duplicate the readings)
- The third injector adds a constant value of 1 to all LIDAR distance values
- Finally, the message is sent to the "scan_fi" channel, to be read by the Turtlebot3 driving simulation
