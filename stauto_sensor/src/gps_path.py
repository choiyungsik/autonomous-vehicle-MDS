#! /usr/bin/env python

import rospy
import tf
import numpy as np
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

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

def odom_callback(data):
    global step_gps, odom_x, odom_y
    step_gps+=1

    odom_x = data.pose.pose.position.x
    odom_y = data.pose.pose.position.y


def euler_to_quaternion(yaw, pitch, roll):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def pub_nav(lat,lon):

    gpsmsg.header.stamp = rospy.Time.now()
    gpsmsg.header.frame_id = "GPS_link"
    gpsmsg.latitude=lat
    gpsmsg.longitude=lon
    gpsmsg.position_covariance_type=0
    navs_pub.publish(gpsmsg)
    #print(gpsmsg)

def pub_path(theta):
    global step_gps
    qx, qy, qz, qw = euler_to_quaternion(theta, 0, 0)



    pathmsg.header.seq = step_gps
    pathmsg.header.stamp = rospy.Time.now()
    pathmsg.header.frame_id = "GPS_link"

    pose = PoseStamped()

    pose.header.seq = pathmsg.header.seq
    pose.header.stamp = pathmsg.header.stamp
    pose.header.frame_id = pathmsg.header.frame_id

    pose.pose.position.x = odom_x
    pose.pose.position.y = odom_y
    pose.pose.position.z = 0

    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    pathmsg.poses.append(pose)

    path_pub.publish(pathmsg)



    print(pathmsg)

if __name__ == '__main__':

    rospy.init_node('gps_path')
    listener = tf.TransformListener()

    rospy.Subscriber("odom",Odometry,odom_callback)

    navs_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=1)
    path_pub = rospy.Publisher('/path', Path, queue_size=1)

    arg_name = rospy.get_param("~arg_name","gps_data/")

    f = open(arg_name + "gps_data_seoultech.txt","r")

    gps_data = f.readlines()
    step_gps=0
    odom_x=0
    odom_y=0
    theta=[]

    gpsmsg=NavSatFix()
    pathmsg=Path()
    pose = PoseStamped()

    while not rospy.is_shutdown():
        #s(_,rot) = listener.lookupTransform('pose', 'base_link', rospy.Time(0))
        #yaw = -tf.transformations.euler_from_quaternion(rot)[2]
        if (step_gps<len(gps_data)-1):

            gps_n = gps_data[step_gps].split(',')
            gps_n_1 = gps_data[step_gps+1].split(',')
            gps_n = [float(gps_n[0]), float(gps_n[1])]
            gps_n_1 = [float(gps_n_1[0]), float(gps_n_1[1])]


            pub_nav(gps_n[0], gps_n[1]) #from LL to UTM

            gps_n = convert_degree_to_meter(gps_n[0],gps_n[1])
            gps_n_1 = convert_degree_to_meter(gps_n_1[0],gps_n_1[1])
            gps_n[0] = (gps_n[0] - 460000)/100
            gps_n[1] = (gps_n[1] - 383000)/100
            gps_n_1[0] = (gps_n_1[0] - 460000)/100
            gps_n_1[1] = (gps_n_1[1] - 383000)/100

            theta.append(atan2(gps_n_1[1]-gps_n[1], gps_n_1[0]-gps_n[0]))

            pub_path(theta[step_gps])
            #Line = sqrt((gps_n[0]-trans[0])**2 + (gps_n[1]-trans[1])**2)
        else:
            pass
