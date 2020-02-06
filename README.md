# Seoultech Autonomous car

## Commands

### USB cam

``roslaunch usb_cam usb_cam-test.launch ``


### Velodyne lidar

``roslaunch velodyne_pointcloud VLP16_points.launch``


### IMU

``rosrun  pangyo_control get_imu 
``

### GPS

``rosrun  pangyo_control gps_data_pangyo.py 
``

### AMCL
``roslaunch stauto_sensor HyphaROS_MiniCar_Racing.launch``

### wheel_odom
``rosrun stauto_sensor enc_odom.py ``

## ROSBAG save

``rosbag record -a ``
