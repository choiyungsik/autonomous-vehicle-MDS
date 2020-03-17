#! /usr/bin/env python

import rospy
import tf
import numpy as np
import rospkg
import math
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from gps_common import *
import copy
from math import *

def path_callback(data):
    global gps_theta, gps_n_1
    gps_theta=data.x
    gps_n_1[0]=data.y
    gps_n_1[1]=data.z
    #print(gps_theta,gps_n_1[0],gps_n_1[1])

def imu_callback(data):
    global imu_theta
    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    roll, pitch, yaw = euler_from_quaternion (orientation_list)

    imu_theta=yaw*(180/np.pi)
    #print(yaw)

def gps_callback(data):
    global lat, lon

    #print(odom_x)
    lat = data.latitude
    lon = data.longitude

if __name__ == '__main__':

    rospy.init_node('gps_control')
    listener = tf.TransformListener()

    rospy.Subscriber("/path",Point,path_callback)
    rospy.Subscriber("/gps/fix",NavSatFix,gps_callback)
    rospy.Subscriber("/imu/data",Imu,imu_callback)

    #navs_pub = rospy.Publisher('/fix', NavSatFix, queue_size=1)
    ackermann_pub = rospy.Publisher('/pure_pursuit/ackermann_cmd', AckermannDriveStamped, queue_size=10)
    #goal_pub = rospy.Publisher('/pure_pursuit/goal', PoseStamped, queue_size=1)

    ackermann=AckermannDriveStamped()

    imu_theta=0.
    gps_theta=0.
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
            if(((-90>=imu_theta>=-180+error_yaw) or (180+error_yaw>=imu_theta>=90)) and ((-90>=gps_theta>=-180+error_yaw) or (180+error_yaw>=gps_theta>=90))):
                if((imu_theta>=0) and (gps_theta<0)):
                    goal_theta=-((180-imu_theta)+(180+gps_theta))
                elif((imu_theta<0) and (gps_theta>=0)):
                    goal_theta=(180+imu_theta)+(180-gps_theta)
            else:
                goal_theta=imu_theta-gps_theta

        if((gps_n_1[0]-lat)==0):
            prize_theta=0
        else:
            prize_theta=math.tan((gps_n_1[0]-lat)/(gps_n_1[1]-lon))*180/np.pi

        goal_theta=goal_theta+error_yaw #yaw offset

        final_angle = prize_theta*0.2 + goal_theta*0.8




        if (final_angle>=28):
            final_angle=28
        elif(final_angle<=-28):
            final_angle=-28




        print("imu",int(imu_theta),"gps",int(gps_theta),"goal",int(goal_theta),"pirze",int(prize_theta))
        ackermann.drive.speed = 2
        ackermann.drive.steering_angle = -goal_theta*np.pi/180

        ackermann_pub.publish(ackermann)
    else:
        pass

#!/usr/bin/env python
