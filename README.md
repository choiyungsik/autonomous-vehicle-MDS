# autonomous-vehicle-MDS


## Commands

### Velodyne lidar & IMU & ERP_to_PC & GPS 

``roslaunch stauto_senor stauto_setting.launch``

### USB cam

``roslaunch usb_cam usb_cam-test.launch ``


### Velodyne lidar

``roslaunch velodyne_pointcloud VLP16_points.launch``


### IMU

``rosrun  stauto_sensor get_imu 
``

### Getting ERP42 information (encoder, steer, speed, brake, gear)

``rosrun  stauto_control ERP42_to_PC
``

### GPS

``rosrun  stauto_sensor get_gps_data.py 
``

### GPS Save

``rosrun stauto_sensor gps_data_save.py 
``

### GPS Path

``rosrun stauto_sensor gps_path.py 
``

### localization

``roslaunch stauto_sensor STauto_Racing.launch``

param

``pub_wheel_odom_tf:=true``: wheel_odom tf publish 결정

``use_amcl``:amcl 사용결정

## ROSBAG save

``rosbag record -a ``


## ROSBAG play

``rosbag play -($rosbag name).bag --clock``

## you need hyparos file
under /src file 

``git clone https://github.com/Hypha-ROS/hypharos_minicar.git``

## Protocol

![img](./docs/Protocol_set.png)

<<<<<<< HEAD
=======


>>>>>>> 38e7c83baea853f04c743eb35a4fa1d5d29d0f25
