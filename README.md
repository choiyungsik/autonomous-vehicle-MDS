# autonomous-vehicle-MDS


## Commands

### USB cam

``roslaunch usb_cam usb_cam-test.launch ``


### Velodyne lidar

``roslaunch velodyne_pointcloud VLP16_points.launch``


### IMU

``rosrun  stauto_sensor get_imu ``


### Getting ERP42 information (encoder, steer, speed, brake, gear)

``rosrun  stauto_control ERP42_to_PC``

### GPS

``rosrun  stauto_sensor get_gps_data.py ``

### AMCL

``roslaunch stauto_sensor HyphaROS_MiniCar_Racing.launch ``

## ROSBAG save

``rosbag record -a ``


## ROSBAG play

``rosbag play -($rosbag name).bag --clock --topic /velodyne_points /imu/data ``

## LEGO-LOAM
``roslaunch lego_loam run.launch ``

실시간 확인시 `use_sim_time` 값 false로 변경

## navsat_transform
``roslaunch robot_localization navsat_transform_template.launch ``

params: magnetic_declination_radians: -1.3731438391195196491 변경 요망
        publish_filtered_gps: true

## Protocol

![img](./docs/Protocol_set.png)
