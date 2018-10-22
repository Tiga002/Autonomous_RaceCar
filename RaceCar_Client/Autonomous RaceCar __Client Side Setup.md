# Autonomous RaceCar -- Client Side Setup

This guide will clarify and explain the normal setup of ROS Navigation Stack. Focusing on how different packages and nodes work together, and how they communicate with each other. 

ROS Navigation Stack is a classic stack within the ROS coummunity. It is built up by different packages. Different packages communicate with each other using a standardise message types. To facilitate autonomus navigation using ROS Navigation Stacks, we have to follow its system architecture. 

> As a pre-requisite for navigation stack use, the robot must be running ROS, have a [tf](http://wiki.ros.org/tf) transform tree in place, and publish sensor data using the correct ROS [Message types](http://wiki.ros.org/msg). Also, the Navigation Stack needs to be configured for the shape and dynamics of a robot to perform at a high level.

To grasp the overview and full documentation as well as official setup guide, you better visit the ROS wiki:
http://wiki.ros.org/navigation

![attachment:overview_tf.png](http://wiki.ros.org/navigation/Tutorials/RobotSetup?action=AttachFile&do=get&target=overview_tf_small.png)

As mentioned before, we have to setup our system as the picture above. It takes in information from odometry and sensor streams and outputs velocity commands to send to a mobile base. 

> The navigation stack assumes that the robot is configured in a particular manner in order to run. The diagram above shows an overview of this configuration. The white components are required components that are already implemented, the gray components are optional components that are already implemented, and the blue components must be created for each robot platform. 

##Setting up the Transformation Tree TF

http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF

The First thing to setup is the Transformation Frame along the Lidar and the Base_Link.

![simple_robot.png](http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF?action=AttachFile&do=get&target=simple_robot.png)

![tf_robot.png](http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF?action=AttachFile&do=get&target=tf_robot.png)

```base_link``` is the frame id which refer to the center of the RaceCar. 

```base_laser``` is the frame id which refer to the position of the Lidar.

Since there is a distance between Lidar and the Center of the RaceCar. The distance data captured from the Lidar has to transform respect to the centre of the racecar before use. 

> To define and store the relationship between the "base_link" and "base_laser" frames using tf, we need to add them to a transform tree. Conceptually, each node in the transform tree corresponds to a coordinate frame and each edge corresponds to the transform that needs to be applied to move from the current node to its child. Tf uses a tree structure to guarantee that there is only a single traversal that links any two coordinate frames together, and assumes that all edges in the tree are directed from parent to child nodes.

In ROS wiki, there is a code example of publishing the transform information. However, fortunately we don't need to implement it by ourselves, we can establish the TF tree by ROS's ```tf``` package built-in nodes.

```bash
<node pkg="tf" type="static_transform_publisher" name="base_to_laser_broadcaster" 
args="0 0 0.5 0 0 0 base_link laser 100" />

# 0 0 0.5 is the translational transform
# 0 0 0 is the orientation transform <roll, pitch, yaw>
# base_link is the parent frame
# laser is the laser frame
```

This built-in node is implement through ```racecar_navigation/launch/RaceCar_Configuration.launch```

## Publishing Sensor Stream Over ROS

http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors

Next thing to do is to setup the Lidar sensor stream. Lidar senses the distance of objects/obstacles around the RaceCar.

> Publishing data correctly from sensors over ROS is important for the Navigation Stack to operate safely. If the Navigation Stack receives no information from a robot's sensors, then it will be driving blind and, most likely, hit things. There are many sensors that can be used to provide information to the Navigation Stack: lasers, cameras, sonar, infrared, bump sensors, etc. However, the current navigation stack only accepts sensor data published using either the `sensor_msgs/LaserScan`Message type or the `sensor_msgs/PointCloud` Message type. 

Fortunately, RP_Lidar is well supported by ROS. We dont have to write our own code to transform the raw data as ```sensor/msgs/LaserScan``` message type. We only have to launch the following launch file. Lidar sensor stream will be published to the navigation stack. Same as before, it is implement through ```racecar_navigation/launch/RaceCar_Configuration.launch```. Therefore, working with the ```static_transform_pubisher``` mentioned above, the Lidar Sensor Stream will be transformed respect to the base_link of the RaceCar. 

```bash
<include file="${find rplidar_ros}/launch.rplidar.launch" >
```

```html
<launch>
  <node name="rplidarNode"          pkg="rplidar_ros"  type="rplidarNode" output="screen">
  <param name="serial_port"         type="string" value="/dev/ttyUSB2"/>
  <param name="serial_baudrate"     type="int"    value="115200"/>
  <param name="frame_id"            type="string" value="laser"/>
  <param name="inverted"            type="bool"   value="false"/>
  <param name="angle_compensate"    type="bool"   value="true"/>
  </node>
</launch>
```

*Sometimes the USB Port Number is changed to /dev/ttyUSB1*

## Publishing Odometry Information over ROS

In my opinion, it is the second most tricky part of the system. Apart from wheel encoder, IMU and Lidar is also used to calculate the odometry. Extended Kalman Filter is used to fuse these three sensors. 

Before explaining further detail, lets launch different sesors first.

###Reading the wheel encoder data

***RaceCar Client Side***

```bash 
<node pkg="serial_communication" type="read_encoder.py" name="encoder_publisher" />  
```

The encoder reading is published in a topic named ```/encoder```. But this is not enough, we have to repack it to the ```nav_msgs/Odometry``` message type. Following the Code example  in http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom . However, it is still in an experimental stage, as even though without the wheel encodering value, the odometry can still be traced reasonably. The expriment code is placed at the ```Autonomus_RaceCar/RaceCar_Client/test_codes``` .

###Reading the IMU

***RaceCar Client Side***

```bash
<include file="$(find razor_imu_9dof)/launch/razor-pub.launch" />
```

RaceCar uses the Sparkfun Razor 9DOF IMU SEN-10736, where Razor officially supporting the ROS Implementation. We dont actually need to write the firmware by ourselves to read the IMU data. The IMU data will be published with a topic named: ```/imu```

By using there official ROS package : razor_imu_9dof. We can gain the accleration on X,Y,Z direction and roll, pitch, yaw angle easily through ROS. More information and calibration steps can be found in ...

ROS wiki : http://wiki.ros.org/razor_imu_9dof

### Lidar for Odom

We can also use the Lidar Sensor Stream to measure the planar movement of RaceCar. 

The algorithm: 
Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA 2016 
Available at: http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/papers/217

To implement this algorithm we have to use the ```rf2o_laser_odometry```package. For more detailed information, please visit : http://wiki.ros.org/rf2o_laser_odometry and https://github.com/MAPIRlab/rf2o_laser_odometry.

> Requirement to use this package:
>
> RF2O core is implemented within the **Mobile Robot Programming Toolkit** [MRPT](http://www.mrpt.org/), so it is necessary to install this powerful library (see instructions [here](http://www.mrpt.org/download-mrpt/)) So far RF2O has been tested with the Ubuntu official repository version (MRPT v1.3), and we are working to update it to MRPT v.1.9

#### Installation Guide -- MRPT

```bash
sudo add-apt-repository ppa:joseluisblancoc/mrpt-1.3
sudo apt-get update
sudo apt-get install libmrpt-dev mrpt-apps
```

#### Launching Guide

```bash
<include file="$(find rf2o_laser_odometry)/launch/rf2o_laser_odometry.launch" />
```

It will subscribe the Lidar Sensor Stream Data, and publishing a /odom in ```nav_msgs/Odometry``` message type.

## Sensor Fusion -- robot_localization

As mentioned before, Extended Kalman Filter is used to filter different sensor sources, as to gain a more precise odometry. 

A popular package we can use to implement the Sensor Fusion : Robot_localization.
Official ROS wiki: http://wiki.ros.org/robot_localization
Official Documentation: http://docs.ros.org/melodic/api/robot_localization/html/index.html
A very good tutorial: https://github.com/methylDragon/ros-sensor-fusion-tutorial/blob/master/01%20-%20ROS%20and%20Sensor%20Fusion%20Tutorial.md

Robot_localization package is a very powerful package, it can fuse unlimited sensor sources. Top 3 features are:

1. Two choices of kalman filter to use (EKF and UKF)
2. Fusing any number of input sensor and pose estimate data sources
   - As long as they have valid message types (odometry, imu, pose, and twist)
3. Integration of 15 states:
   - Position: x, y, z
   - Orientation: yaw, pitch, roll
   - Linear Velocity: x', y', z'
   - Angular Velocity: yaw', pitch', roll'
   - Linear Acceleration: x'', y'', z''

Before discussing how to use the robot_localization package to fuse the sensor, there are some fundamental knowledge have to clarify.

#### Map, Odom, and Base_link

There are two kinds of pose estimates, one for the robot's local position (which is continuous and drifts over time), and one of the robot's estimated position globally (which is discontinuous but more accurate in the long run).

And these pose estimates affect **different** transforms between the three coordinate frames of the map, odom and base_link frames.

![map, odom, and base_link](https://github.com/methylDragon/ros-sensor-fusion-tutorial/raw/master/assets/1_2.png)

1. **Map --> Odom** 
   - **Accounts for the robot's global pose**
   - Is **discontinuous** (jumps around)
   - Is published by the **AMCL node** (or any other global pose estimators!)
2. **Odom --> Base_Link**
   - **Accounts for the robot's local pose**
   - Is **continuous**, but drifts over time due to sensor drift, errors accumulates over time
   - Is published by the **odometry node** (which should be taking into account encoders, IMUs and other sensors like laser scan matching nodes, etc.)

### Practical Roadmap 

1. Prepare the Data
2. Configure the Kalman Filter Parameters
3. Run and Tune!

#### Prepare the Data

Official guidelines: http://docs.ros.org/kinetic/api/robot_localization/html/preparing_sensor_data.html#

We have to follow some of the ROS Standards to prepare the data. They are:

- [REP-103 (Standard Units of Measure and Coordinate Conventions)](http://www.ros.org/reps/rep-0103.html)
  - If your robot is moving counter-clockwise, yaw should be positive.
  - If your robot is moving forward, x should be positive.
- [REP-105 (Coordinate Frame Conventions)](http://www.ros.org/reps/rep-0105.html)
  - Earth --> Map --> Odom --> Base_Link

Message types can be used as inputs to robot_localization nodes:

- nav_msgs/Odometry
  - Position, Orientation, and Velocity
- geometry_msgs/PoseWithCovarianceStamped
  - Position and Orientation
- geometry_msgs/TwistWithCovarianceStamped
  - Velocity
- sensor_msgs/Imu
  - Angular Velocity and Linear Acceleration

#### Configure the Kalman Filter Parameters

The parameters and setting are stored in the configure file ```ekf_fuse_odom.yaml``` in the path ```RaceCar_Client/catkin_ws/src/robot_localization/params``` .

First thing to configure is the frame arrangement. We follow the ROS Standards REP-105.

If fusing odometry data (scan matching, LIDAR, encoders, etc.):

```yaml
map_frame: map              # Defaults to "map" if unspecified
odom_frame: odom            # Defaults to "odom" if unspecified
base_link_frame: base_link  # Defaults to "base_link" if unspecified
world_frame: odom           # Defaults to the value of odom_frame if unspecified
```

 If fusing global pose estimates (AMCL, beacons, etc.)

```yaml
map_frame: map              # Defaults to "map" if unspecified
odom_frame: odom            # Defaults to "odom" if unspecified
base_link_frame: base_link  # Defaults to "base_link" if unspecified
world_frame: map           # Defaults to the value of odom_frame if unspecified
```

Then we have to specify the sensor inputs and configure it one by one.

```yaml
# Odometry From Lidar Match Scanning
odom0: odom_rf2o  # topic published by rf2o_laser_odometry
# Configure the data to use for each sensor
odom0_config: [true,  true,  true,  # x_pos   , y_pos    , z_pos
               false, false, true,  # roll    , pitch    , yaw,
               true, true, true,    # x_vel   , y_vel    , z_vel,
               false, false, true,  # roll_vel, pitch_vel, yaw_vel,
               false, false, false] # x_accel , y_accel  , z_accel]
# IMU 
imu0: imu  #Topic published by razor_imu_9dof
imu0_config: [false, false, false,
              true,  true,  true,
              false, false, false,
              true,  true,  true,
              true,  true,  true]

```

Then we have to **Specify 2D Mode**!!! This is very important

```yaml
two_d_mode: true
```

To launch the ekf node, there is a launch file in ```RaceCar_Client/catkin_ws/src/robot_localization/launch/ekf_fuse_odom.launch```

```html
<launch>
  <node pkg="robot_localization" type="ekf_localization_node" name="ekf_se" clear_params="true">
  <rosparam command="load" file="$(find robot_localization)/params/ekf_fuse_odom.yaml" />
   <!--  Placeholder for output topic remapping -->
    <remap from="odometry/filtered" to="odom"/>
  </node>
</launch>
```

The Odometry will be fused and published as a topic named in ```/odom``` now !! It will then feed in to the ROS Navigation Stack!

## Launch Everything Together

Til Now, we can see we have setup all the required inputs of the navigation stack. These packages and nodes are running on the RaceCar Client Side. Therefore, I group all of them in a same launch file ```RaceCar_Client/catkin_ws/src/racecar_navigation/launch/RaceCar_Configuration.launch```

```html
<launch>
  <!-- Sensors -->
    <include file="$(find rplidar_ros)/launch/rplidar.launch" />
    <include file="$(find razor_imu_9dof)/launch/razor-pub.launch" />
  <!-- Odometry -->
    <include file="$(find rf2o_laser_odometry)/launch/rf2o_laser_odometry.launch" />
    <include file="$(find robot_localization)/launch/ekf_fuse_odom.launch" />
  <!-- TF Configuration -->
     <node pkg="tf" type="static_transform_publisher" name="base_to_laser_broadcaster" args="0 0 0.5 0 0 0 base_link laser 100"/>
  <!-- Read Wheel Encoder -->
     <node pkg="serial_communication" type="read_encoder.py" name="encoder_publisher" />  
</launch>
```









