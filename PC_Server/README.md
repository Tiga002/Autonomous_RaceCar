# Autonomous_RaceCar_Server

**This ROS Workspace is located in ther Server PC.**

## To Launch the ROS Navigation

1. On RaceCar Client Side
```
roslaunch racecar_navigation RaceCar_Configuration.launch
rosrun serial_communication send_command.py
```
2. On PC Server Side
```
roslaunch racecar_navigation RaceCar_Navigation_PID.launch
```
***the launch files is located at***
```
catkin_ws/src/racecar_navigation/launch
```
