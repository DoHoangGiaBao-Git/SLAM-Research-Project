# SLAM-Research-Project
Repository For SLAM Research

## Run teleop (keyboard)
From the workspace root, run the keyboard teleop command below to pubish velocity command using keyboard (turtlebot4 can be controlled with a controller so this command is not necessary when running turtlebot4):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
## Initialize turtlebot4
From the workspace root, launch `robot.launch.py`:
```bash
ros2 launch robot_model robot.launch.py mode:=real
```
This will only initialize turtlebot4 and open Rviz2.
## Run Gazebo Simulation
From the workspace root, run `robot.launch.py`:
```bash
ros2 launch robot_model robot.launch.py mode:=sim
```
This will initialize robot model specified in `robot_description`, open Rviz2 and Gazebo simulation. The robot model, sensors and physics will follow Gazebo simulation.