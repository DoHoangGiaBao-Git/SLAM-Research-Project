# SLAM-Research-Project
Repository For SLAM Research

## Run teleop (keyboard)
From the workspace root, run the keyboard teleop command below to publish velocity commands remapped to `/robot1/cmd_vel`:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel   
```
This will start the teleop node and remap the local `/cmd_vel` topic to `/robot1/cmd_vel` so your robot or simulator receives the commands.
## Initialize robot
From the workspace root, run `robot.launch.py`:
```bash
ros2 launch robot_model robot.launch.py mode:=real
```
This will only initialize robot and open Rviz2.
## Start Gazebo
From the workspace root, run `robot.launch.py`:
```bash
ros2 launch robot_model robot.launch.py mode:=sim
```
This will initialize robot, open Rviz2 and Gazebo simulation. The robot, sensors and physics will follow Gazebo simulation.