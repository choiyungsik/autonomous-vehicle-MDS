#! /usr/bin/env python

import rospy
import tf
import numpy as np
import rospkg
import math
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from geometry_msgs.msgs import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from gps_common import *
import copy
from math import *


def imu_callback(data):
    global imu_theta
    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    roll, pitch, yaw = euler_from_quaternion (orientation_list)

    imu_theta=yaw*(180/np.pi)
    #print(yaw)

def local_path_callback(data):
    global local_path

    for i in range(len(data)):
        local_path[i][0] = data.poses[i].pose.position.x
        local_path[i][1] = data.poses[i].pose.position.y
    

def robot_position_callback(data):
    global robot_position

    robot_position[0] = data.pose.position.x
    robot_position[1] = data.pose.position.y   


if __name__ == '__main__':

    rospy.init_node('control')
    listener = tf.TransformListener()

    rospy.Subscriber("/local_path",Path,local_path_callback)
    rospy.Subscriber("/robot_position",PoseStamped,robot_position_callback)
    rospy.Subscriber("/imu/data",Imu,imu_callback)



    ackermann_pub = rospy.Publisher('/ackermann_cmd_state', AckermannDriveStamped, queue_size=10)

    ackermann=AckermannDriveStamped()

    imu_theta=0.
    local_path =np.zeros((4,2))
    robot_position= []
    gps_theta=0.
    pure_pursuit_theta=0.
    gps_n_1=[0,0]
    lat=0
    lon=0
    error_yaw=0

    rospy.sleep(1)

    init_time=rospy.Time.now()

    while not rospy.is_shutdown():


        if((int(imu_theta)*int(gps_theta))>=0.):
            goal_theta=imu_theta-gps_theta
        else:
            if(((-90>=imu_theta>=-180) or (180>=imu_theta>=90)) and ((-90>=gps_theta>=-180) or (180>=gps_theta>=90))):
                if((imu_theta>=0) and (gps_theta<0)):
                    goal_theta=-((180-imu_theta)+(180+gps_theta))
                elif((imu_theta<0) and (gps_theta>=0)):
                    goal_theta=(180+imu_theta)+(180-gps_theta)
            else:
                goal_theta=imu_theta-gps_theta


        print(pure_pursuit_theta)

        goal_theta=goal_theta+error_yaw #yaw offset

        final_angle = pure_pursuit_theta*1.0 + goal_theta*0




        if (final_angle>=28):
            final_angle=28
        elif(final_angle<=-28):
            final_angle=-28


        print("imu",int(imu_theta),"gps",int(gps_theta),"goal",float(pure_pursuit_theta)*180/np.pi,"final",int(final_angle))
        ackermann.drive.speed = 3
        ackermann.drive.steering_angle = -final_angle*np.pi/180

        ackermann_pub.publish(ackermann)
    else:
        pass
