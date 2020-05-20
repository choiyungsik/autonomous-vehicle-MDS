#! /usr/bin/env python

import rospy
import tf
import numpy as np
import rospkg
import math
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
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

def speed_callback(data):
    global speed

    speed = data.data/10

def local_path_callback(data):
    global local_path

    for i in range(len(data.poses)):
        local_path[i][0] = data.poses[i].pose.position.x
        local_path[i][1] = data.poses[i].pose.position.y
    #print(local_path)
    #print(sqrt((local_path[0][0]-local_path[3][0])**(2) + (local_path[0][1]-local_path[3][1])**(2)))
def cur_gps_position_callback(data):
    global cur_gps_position

    cur_gps_position[0] = data.latitude
    cur_gps_position[1] = data.longitude
    #print(cur_gps_position)

if __name__ == '__main__':

    rospy.init_node('control')
    listener = tf.TransformListener()

    rospy.Subscriber("/local_path",Path,local_path_callback)
    rospy.Subscriber("/gps/current_robot_position",NavSatFix,cur_gps_position_callback)
    rospy.Subscriber("/imu/data",Imu,imu_callback)
    rospy.Subscriber("/ERP42_speed",Float32,speed_callback)

    start_yaw = rospy.Publisher('yaw_error', Float32, queue_size=10)
    ackermann_pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=10)

    ackermann=AckermannDriveStamped()

    imu_theta=0.
    local_path =np.zeros((4,2))
    cur_gps_position= [0,0]
    gps_theta=0.
    speed=0

    error_yaw=0
    start_yaw_sign=True
    camera_theta=0.

    going_gps_n=[0,0]
    going_gps_n1=[0,0]
    going_gps_n2=[0,0]
    going_gps_n3=[0,0]
    going_gps=[0,0]

    max_speed=2.5
    min_speed=1.5
    rospy.sleep(1.5)

    init_time=rospy.Time.now()

    while not rospy.is_shutdown():

        going_gps[0]=cur_gps_position[0]
        going_gps[1]=cur_gps_position[1]

        #going_gps_n2[0]=(utm_next_gps[0] - 460000)/100
        #going_gps_n2[1]=(utm_next_gps[1] - 383000)/100

        #test

        going_gps_n[0]=local_path[0][0]
        going_gps_n[1]=local_path[0][1]
        going_gps_n1[0]=local_path[1][0]
        going_gps_n1[1]=local_path[1][1]
        going_gps_n2[0]=local_path[2][0]
        going_gps_n2[1]=local_path[2][1]
        going_gps_n3[0]=local_path[3][0]
        going_gps_n3[1]=local_path[3][1]

        going_gps_theta = atan2(going_gps_n3[1]-going_gps[1], going_gps_n3[0]-going_gps[0])*180/np.pi
        #print(going_gps_theta,imu_theta)
        if (start_yaw_sign==True):
            start_theta = atan2(going_gps_n2[1]-going_gps_n[1], going_gps_n2[0]-going_gps_n[0])*180/np.pi
            print(start_theta)
            start_yaw.publish(-start_theta)
            start_yaw_sign=False

        print(going_gps_theta, imu_theta)
        if((int(imu_theta)*int(going_gps_theta))>=0.):
            alpha=imu_theta-going_gps_theta
        else:
            if(((-90>=imu_theta>=-180) or (180>=imu_theta>=90)) and ((-90>=going_gps_theta>=-180) or (180>=going_gps_theta>=90))):
                if((imu_theta>=0) and (going_gps_theta<0)):
                    alpha=-((180-imu_theta)+(180+going_gps_theta))
                elif((imu_theta<0) and (going_gps_theta>=0)):
                    alpha=(180+imu_theta)+(180-going_gps_theta)
            else:
                alpha=imu_theta-going_gps_theta

        alpha=alpha*(np.pi/180)  # alpha => degree
        #print(alpha)
        #print(local_path[0][0],cur_gps_position[0],local_path[0][1],cur_gps_position[1])
        L=1.3
        Ld=sqrt((local_path[0][0]-cur_gps_position[0])**(2) + (local_path[0][1]-cur_gps_position[1])**(2))

        speed_ld = speed*0.237
        alpha_ld = abs(alpha*180/np.pi)*0.05

        if(alpha_ld>=2):
            alpha_ld=2

        ld = speed_ld+3-alpha_ld
        #print(speed_ld,alpha_ld, ld)
        gps_theta=atan(2*L*sin(alpha)/ld)*(180/np.pi)

        final_angle = gps_theta*1.0 + camera_theta*0

        if (final_angle>=28):
            final_angle=28
        elif(final_angle<=-28):
            final_angle=-28

        if(abs(final_angle)>=10):
            Speed_linear = (max_speed-min_speed)/28*(28-abs(final_angle))+min_speed
        else:
            Speed_linear=max_speed
        #print(Speed_linear)
        print(Ld, gps_theta, Speed_linear)
        ackermann.drive.speed = Speed_linear
        ackermann.drive.steering_angle = -final_angle*np.pi/180
        ackermann_pub.publish(ackermann)


    else:
        pass
