## Compile

`colcon build [--packages-select robo_fi]`

### Change tb3 src

* `src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/src/turtlebot3_drive.cpp`
* `s/"scan"/"scan_fi"`
* Don't forget to recompile

## Services to run

Inside rocker (to have GUI):
* `source install/setup.bash`
* `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`

Scripts:
* `ros2 run robo_fi base`
* `ros2 run turtlebot3_gazebo turtlebot3_drive`
