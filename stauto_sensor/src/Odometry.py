#!/usr/bin/env python
import serial
import numpy as np
import rospy
import tf
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def pub_odometry(gps, quaternion, vth, speed):
    print(vth, speed)
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"


    # set the position
    odom.pose.pose = Pose(Point(gps[0], gps[1], 0.), Quaternion(*quaternion))

    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(speed, 0, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)


    odom_broadcaster.sendTransform(
        (gps[0], gps[1], 0.),
        quaternion,
        odom.header.stamp,
        "base_link",
        "odom"
    )


def cur_gps_position_callback(data):
    global cur_gps_position

    cur_gps_position[0] = data.latitude
    cur_gps_position[1] = data.longitude

def imu_callback(data):
    global imu_vth, imu_quaternion
    imu_quaternion = [0, 0, data.orientation.z, data.orientation.w]
    imu_vth = data.angular_velocity.z
def speed_callback(data):
    global speed

    speed = data.data/10

if __name__ == '__main__':
    imu_quaternion=[0,0,0,0]
    cur_gps_position=[0,0]
    speed=0
    imu_vth=0
    rospy.init_node('Odometry')

    #listener = tf.TransformListener()

    rospy.Subscriber("/gps/current_robot_position",NavSatFix,cur_gps_position_callback)
    rospy.Subscriber("/imu/data",Imu,imu_callback)
    rospy.Subscriber("/ERP42_speed",Float32,speed_callback)
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom = Odometry()
    odom_broadcaster = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        pub_odometry(cur_gps_position, imu_quaternion, imu_vth, speed)



    else:
        pass
