# Autonomous RaceCar

RaceCar is a ROS compatible robot that is capable for Simultaneous Localization and Mappting as well as Autonomous Indoor Navigation. Different Algorithms are adapted, such as Monte Carlo Localization, Kalman Filter, Particle Filters, Gmapping, as well as Timed-Elastic-Band local planner. The main architecture of the system is following the standard setup of ROS Navigation Stack, which will be explained later.

The documentation can be separated into 3 main parts. Here is the main overview of the system and the command to launch the RaceCar. A more informative explaination will be made in the other 2 documents inside the folder ```</PC_Server>```and ```</RaceCar_Server>```. 



## Hardware Platform

RaceCar is built up from Traxaas RC racecar chasis, which is an ackermann steerling based robot platform. An outrunner motor is mounted at the rear of the RaceCar, with a gear box attached. It powers up the forward and backward movement of the RaceCar (*is denoted as x-direction*). While a servo motor is attached at the front, which directs the steerling. For more information about the support of ackermann steerling based robot from ROS, please visit: http://wiki.ros.org/Ackermann%20Group. 

### Power System

There are 2 power sources for powering up the RaceCar. One is the NiMH Power Cell with 4200MaH and 8.4V output. It powers the Electronic Speed Controller, which controls and deliver power to the outrunner motor, Servo Motor. The another power source is an 20V Portable Power Bank. It powers up the Nvidia TX1,  and different sensors.

### Processing Unit

The main processing board is Nvidia TX1. Ubuntu 16.04 and ROS is installed on TX1. It is the "brain" of the RaceCar. All sensors is connected to TX1 via USB-Port. It gathers all the raw data from different sensors and process them as useful information. 

Meanwhile it connects the PC_Server via Wifi-LAN. Although the processing ability of TX1 is far more stronger than Teensy, Arduino, Raspberry Pi, it still cannot afford the high processing demand from the real time global and local path planning. Therefore, once the TX1 gathers the infromation from the sensor, it will pass the information to the PC_Server for doing the global and local path planning. Then the PC_Server will pass the control signal to TX1, then TX1 pass the message to the base microcontroller, Teensy. 

To Connect the TX1 with PC, we have to amend the ~/.bashrc on both platform.

**PC_Server Side** 

```bash
export ROS_HOSTNAME = ip_address of the PC Server
export ROS_MASTER_URI = http://${ROS_HOSTNAME}:11311
```

**RaceCar_Client Side**

```bash
export ROS_HOSTNAME = ip_address of the RaceCar
export ROS_MASTER_URI = http://ip_address of the PC Server:11311
```

Teensy 3.6 is the microcontroller outputing control signal to the ESC and servo motor.  Servo Motor is connecting at Pin9, while ESC signal is connected to the Pin10. A ground cable has to be connected with the Traxaas TQ. 

Arduino Nano is used to prototype the DIY wheel encoder. Since ROS Navigation stack and PID control inside the base controller needs a precise odometry and speed, a wheel encoder is more preferable. 
As the outrunner motor does not equipped with an optical wheel encoder, I prototype it by using a Hall-Effect Sensor and a small magnet. Whenever the Hall-Effect sensor sense the magnet is opposite itself, the signal will be raised. Then an interrupt service is established inside Arduino Nano. Whenever the signal is rasied, the counter will increments. Every 0.5 second, Arduino Nano will output the number of counts within half-seconds, then reset the counter to zero again. Therefore, we can calculated the number of revolution of the wheels thus to find the speed of the RaceCar. 

Both Teensy 3.6 and Arduino Nano is connected with TX1 via USB-Port as ```/dev/ttyACM0``` and ```/dev/ttyUSB0``` respectively. Data Communication between them and TX1 is facilitated by PySerial. Repective Codes are inside ```/RaceCar_Client/catkin_ws/src/serial_communication```. 

### Sensors

RaceCar is equipped with different sensors as to measure the surrounding environment. It equips with:

- USB Camera
- LiDar
- 9 DOF IMU
- Wheel Encoder

## Launching the RaceCar

### Simultaneous Localization and Mapping

Before letting RaceCar to navigate indoor by itself, we have to build a probability-based 2D costmap for it first. There are many SLAM solutions available with ROS supported. Gmapping is used in our case. 

**PC_Server Side**

```bash
$ roslaunch racecar_navigation RaceCar_SLAM_Server.launch
```

**RaceCar_Client Side**

```bash
$ roslaunch racecar_navigation RaceCar_SLAM_Client.launch
# Wait for 10 seconds 
$ rosrun serial_communication send_command.py
```

Then, we can control the RaceCar with the XBOX wireless controller. Axis1(upper left) is used to control the forward speed. Axis2(bottom right) is used to control the steerling. 

Then we can drive the RaceCar within the room or arround the corridor. The map is generated instantly and we can view the map on the rviz which is running at the PC_Server side. Once we think the map has covered the area we wish to cover, run the following command at PC_Server Side :

```bash
cd <path_you_want_to_save_the_map>
$ rosrun map_server map_saver -f name_of_the_map
```

### Indoor Navigation

Once we have built the map, RaceCar can navigates by itself. But first we have to make sure the launch files is ready. Such as the file path of the map is correct, etc.

```<PC_Server/catkin_ws/src/racecar_navigation/launch/RaceCar_Navigation_PID.launch```

```bash
<launch>
    <master auto ="start" />
     <!-- Run the Map Server -->
        <node name="map_server" pkg="map_server" type="map_server" args="$(find racecar_navigation)/maps/ROOM_2.yaml" /> <!-- File path of the map -->
    <!-- Run AMCL -->
        <include file="$(find racecar_navigation)/launch/RaceCar_AMCL.launch" />
    <!-- Run Move_Base and its related Nodes -->
        <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
      <!-- 2D Costmap : Global and Local -->
        <rosparam file="$(find racecar_navigation)/params/carlike/costmap_common_params.yaml" command="load" ns="global_costmap" />
        <rosparam file="$(find racecar_navigation)/params/carlike/costmap_common_params.yaml" command="load" ns="local_costmap" />
        <rosparam file="$(find racecar_navigation)/params/carlike/local_costmap_params.yaml" command="load" />
        <rosparam file="$(find racecar_navigation)/params/carlike/global_costmap_params.yaml" command="load" />
      <!-- Move_Base Configuration -->
        <rosparam file="$(find racecar_navigation)/params/move_base_params.yaml" command="load" />
      <!-- Local Planner -->
        <rosparam file="$(find racecar_navigation)/params/teb_local_planner_params.yaml" command="load" />
      <!-- Global Planner -->
        <rosparam file="$(find racecar_navigation)/params/base_global_planner_params.yaml" command="load" /> 
        </node> 
   <!-- Run RVIZ for Visualization --> 
        <node pkg="rviz" type="rviz" name="rviz" required="true" args="-d $(find racecar_navigation)/rviz/RaceCar_Navigation_Visualization.rviz" />  
  
    <!-- Ackermann Base Controller with PID -->
 <node pkg="base_controller" type="PID_Base_Controller.py" name="pid_base_controller_node" />  
</launch>
```

Once everything is ready, we can run the following command:

**First: RaceCar Client Side**

```bash
$ roslaunch racecar_navigation RaceCar_Configuration.launch
$ rosrun serial_communication send_command.py
```

**Then: PC_Server Side**

```bash
$ roslaunch racecar_navigation RaceCar_Navigation_PID.launch
```



