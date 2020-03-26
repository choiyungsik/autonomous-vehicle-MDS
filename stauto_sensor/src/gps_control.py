#! /usr/bin/env python

import rospy
import tf
import numpy as np
import rospkg
import math
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from gps_common import *
import copy
from math import *

def convert_degree_to_meter(lat,lon):

    alpha = lat*pi/180
    beta = lon*pi/180

    a = 6377397.155
    f = 1/299.1528128
    b = a*(1-f)
    k = 1
    x_plus = 500000
    y_plus = 200000

    e1 = (a**2-b**2)/a**2
    e2 = (a**2-b**2)/b**2
    alpha0 = 38 *pi/180
    beta0 = (125+0.002890277)*pi/180

    T = pow(tan(alpha),2)
    C = e1/(1-e1)*pow(cos(alpha),2)
    AA = (beta-beta0)*cos(alpha)  #both radian

    N = a/sqrt( 1-e1*sin(alpha)**2 )
    M = a*(alpha*(1-e1/4-3*e1**2/64-5*e1**3/256)-sin(2*alpha)*(3*e1/8+3*e1**2/32+45*e1**3/1024)
        +sin(4*alpha)*(15*e1**2/256+45*e1**3/1024)-35*e1**3*sin(6*alpha)/3072)

    M0 = a*(alpha0*(1-e1/4-3*e1**2/64-5*e1**3/256)-sin(2*alpha0)*(3*e1/8+3*e1**2/32+45*e1**3/1024)
         +sin(4*alpha0)*(15*e1**2/256+45*e1**3/1024)-35*e1**3*sin(6*alpha0)/3072)

    Y = y_plus + k*N*(AA+AA**3*(1-T+C)/6+pow(AA,5)*(5-18*T+T*T+72*C-58*e2)/120)
    X = x_plus + k*(M-M0+N*tan(alpha)*(AA*AA/2 + pow(AA,4)*(5-T+9*C+4*C*C)/24 +
        pow(AA,6)*(61-58*T+T*T+600*C-330*e2)/720))

    return [X,Y]

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

def prize_callback(data):
    global prize_theta

    prize_theta=data.data

if __name__ == '__main__':

    rospy.init_node('gps_control')
    listener = tf.TransformListener()

    rospy.Subscriber("/path",Point,path_callback)
    rospy.Subscriber("/gps/fix",NavSatFix,gps_callback)
    rospy.Subscriber("/imu/data",Imu,imu_callback)
    rospy.Subscriber("/prize",Float32,prize_callback)

    #navs_pub = rospy.Publisher('/fix', NavSatFix, queue_size=1)
    ackermann_pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=10)
    #goal_pub = rospy.Publisher('/pure_pursuit/goal', PoseStamped, queue_size=1)

    ackermann=AckermannDriveStamped()

    imu_theta=0.
    gps_theta=0.
    prize_theta=0.
    gps_n_1=[0,0]
    lat=0
    lon=0
    error_yaw=-1

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
        
        #gps_n_1=convert_degree_to_meter(gps_n_1[0],gps_n_1[1])
        #curl_theta=atan2(gps_n_1[0]-lat, gps_n_1[1]-lon)*180/np.pi

        #if((gps_n_1[1]-lon)==0):
        #    prize_theta=0
        #else:
        #    prize_theta=math.atan((gps_n_1[0]-lat)/(gps_n_1[1]-lon))*180/np.pi
        #    if(prize_theta>=0):
        #        prize_theta=90-prize_theta
        #    else:
        #        prize_theta=-(90+prize_theta)
        print(prize_theta)
        
        goal_theta=goal_theta+error_yaw #yaw offset

        final_angle = prize_theta*0.3 + goal_theta*1




        if (final_angle>=28):
            final_angle=28
        elif(final_angle<=-28):
            final_angle=-28
        




        print("imu",int(imu_theta),"gps",int(gps_theta),"goal",int(goal_theta),"pirze",int(prize_theta))
        ackermann.drive.speed = 2
        ackermann.drive.steering_angle = -final_angle*np.pi/180

        ackermann_pub.publish(ackermann)
    else:
        pass

#!/usr/bin/env python
