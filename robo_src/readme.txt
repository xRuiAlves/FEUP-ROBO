# A study on heuristics for LIDAR-based wall-following AGVs

###### authors: Filipa Durão (up201606640@fe.up.pt), Miguel Duarte (up201606298@fe.up.pt), Rui Alves (up201606746@fe.up.pt)

## Directory Structure

.
├── CMakeLists.txt                  # Compilation rules
├── launch                          # Launch files
│   ├── base.launch
│   └── open_D_shape.launch
├── models                          # Gazebo models
│   ├── open_D_shape_2.dae
│   ├── open_D_shape_3.dae
│   ├── open_D_shape_4.dae
│   ├── open_D_shape_5.dae
│   └── open_D_shape_6.dae
├── package.xml                     # Package specifications file, including dependencies
├── readme.txt                      # README document 
├── scripts                         # Scripts to interact with the package's worlds
│   └── follow_wall.py
└── worlds                          # Gazebo worlds
    ├── base.world
    ├── open_D_shape_0.2.world
    ├── open_D_shape_0.3.world
    ├── open_D_shape_0.4.world
    ├── open_D_shape_0.5.world
    └── open_D_shape_0.6.world

## Requirements

### Software

- ROS Noetic Ninjemys
- Python v3.8
- RosPy
- Catkin (using GNU Make v4.2.1)
- Gazebo v11.0.0

### Package dependencies

The developed package (named "ROBO") requirements are listed in the "package.xml" file. Nevertheless, here are the software versions used:

- TurtleBot3 v1.2.4 (https://github.com/ROBOTIS-GIT/turtlebot3)
- TurtleBot3 Messages v1.0.1 (https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
- TurtleBot3 Simulations v1.2.0 (https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git)

## Compilation

To compile the developed package, run `catkin_make`.

## Executing

### Initializing the world

To execute the package, first launch the world and the turtlebot robot using the launch file:

```
roslaunch robo open_D_shape.launch \
    wall_thickness:=[WALL_THICKNESS] \
    x_pos:=[TURTLEBOT_X_POSITION] \
    y_pos:=[TURTLEBOT_Y_POSITION] \
    z_pos:=[TURTLEBOT_Z_POSITION] \
    angle:=[TURTLEBOT_Z_AXIS_ANGLE]
```

A few considerations regarding the optional arguments:

- The "wall_thickness" argument should take one of the values 0.2, 0.3, 0.4, 0.5 or 0.6
- The "angle" should be specified in radians

### Running the script

Then, you should run the script that makes the turtlebot robot follow the wall:

```
rosrun robo follow_wall.py
```
