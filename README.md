# Seoultech Autonomous car

## Commands

### USB cam

``roslaunch usb_cam usb_cam-test.launch ``


### Velodyne lidar

``roslaunch velodyne_pointcloud VLP16_points.launch``


### IMU

``rosrun  stauto_sensor get_imu 
``

### GPS

``rosrun  stauto_sensor get_gps_data.py ``


### AMCL
``roslaunch stauto_sensor HyphaROS_MiniCar_Racing.launch``


### wheel_odom
``rosrun stauto_control get_encodom_and_control.py ``

## ROSBAG save

``rosbag record -a ``


## Protocol

![img](./docs/Protocol_set.png)
