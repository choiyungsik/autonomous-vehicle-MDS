#!/usr/bin/env python
import serial
import numpy as np
import rospy
from math import *
import tf
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def quaternion_to_euler(w, x, y, z):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1,
                     np.sign(sinp) * np.pi / 2,
                     np.arcsin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    print(yaw*180/np.pi)
    #return roll, pitch, yaw

def pub_odometry(gps, quaternion, vx, vy, vth):
    #print(vth, speed)
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"


    # set the position
    odom.pose.pose = Pose(Point(gps[0], gps[1], 0.), Quaternion(*quaternion))

    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

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
    imu_quaternion = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    imu_vth = data.angular_velocity.z

def speed_callback(data):
    global speed
    speed = data.data/10

def steer_callback(data):
    global steer

    steer = data.data/71

if __name__ == '__main__':
    imu_quaternion=[0,0,0,0]
    cur_gps_position=[0,0]
    speed=0
    steer=0
    imu_vth=0

    lf=1.05/2
    lr=1.05/2

    rospy.init_node('Odometry')

    #listener = tf.TransformListener()

    rospy.Subscriber("/gps/current_robot_position",NavSatFix,cur_gps_position_callback)
    rospy.Subscriber("/imu/data",Imu,imu_callback)
    rospy.Subscriber("/ERP42_speed",Float32,speed_callback)
    rospy.Subscriber("/ERP42_steer",Float32,steer_callback)
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom = Odometry()
    odom_broadcaster = tf.TransformBroadcaster()
    while not rospy.is_shutdown():

        B = atan(lr*tan(steer*(np.pi/180))/(lr+lf))
        vth = speed*sin(B)/lr
        vx = speed*cos(vth+B)
        vy = speed*sin(vth+B)

        pub_odometry(cur_gps_position, imu_quaternion, vx, vy, vth)

        quaternion_to_euler(imu_quaternion[0],imu_quaternion[1],imu_quaternion[2],imu_quaternion[3])

    else:
        pass
