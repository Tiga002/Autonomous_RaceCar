# Guidelines for Setting Up the ROS Navigation Stack 

## Transform Coordinate 
- Transfer the scan-points referenced with /base_laser frame --> /base_link frame
- Details can be found in ROS_Navigation/Tutorials/tf
- But in practice, we do not need to make it bu ourself, 
- we can config the tf relationship inside the /rplidar_ros/launch/hector_slam.launch
```
  <node pkg="tf" type="static_transform_publisher" name="base_to_laser_broadcaster" args="0 0 0.2 0 0 0 /base_link /laser 100"/>
```


