#! /usr/bin/env python

import rospy
import tf
import numpy as np
import rospkg
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from gps_common import *
import copy
from math import *

def gps_callback(data):
    global gps_theta
    gps_theta=data.data

def imu_callback(data):
    global imu_theta
    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    roll, pitch, yaw = euler_from_quaternion (orientation_list)

    imu_theta=yaw*(180/np.pi)-83 #offset
    #print(yaw)

if __name__ == '__main__':

    rospy.init_node('gps_control_1')
    listener = tf.TransformListener()

    rospy.Subscriber("/path",Float32,gps_callback)
    rospy.Subscriber("/imu/data",Imu,imu_callback)

    #navs_pub = rospy.Publisher('/fix', NavSatFix, queue_size=1)
    ackermann_pub = rospy.Publisher('/pure_pursuit/ackermann_cmd', AckermannDriveStamped, queue_size=10)
    #goal_pub = rospy.Publisher('/pure_pursuit/goal', PoseStamped, queue_size=1)

    ackermann=AckermannDriveStamped()

    imu_theta=0.
    gps_theta=0.

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
            
            if (goal_theta>=28):
                goal_theta=28
            elif(goal_theta<=-28):
                goal_theta=-28
             
        print(int(imu_theta),int(gps_theta),int(goal_theta))
        ackermann.drive.speed = 2
        ackermann.drive.steering_angle = -goal_theta*np.pi/180

        ackermann_pub.publish(ackermann)
    else:
        pass

#!/usr/bin/env python
