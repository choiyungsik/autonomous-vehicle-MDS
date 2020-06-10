#! /usr/bin/env python3
#import keyboard
import serial
import socket as soc
import rospy
import time
from std_msgs.msg import UInt32
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

import math
port = str(rospy.get_param("~imu_port","/dev/ttyACM0"))
gps_data_bef = ""


def convert_degree_to_meter(lat_data,lon_data):
    alpha = lat_data*math.pi/180
    beta = lon_data*math.pi/180

    a = 6377397.155
    f = 1/299.1528128
    b = a*(1-f)
    k = 1
    x_plus = 500000
    y_plus = 200000

    e1 = (a**2-b**2)/a**2
    e2 = (a**2-b**2)/b**2
    alpha0 = 38 *math.pi/180
    beta0 = (125+0.002890277)*math.pi/180

    T = pow(math.tan(alpha),2)
    C = e1/(1-e1)*pow(math.cos(alpha),2)
    AA = (beta-beta0)*math.cos(alpha)  #both radian

    N = a/math.sqrt( 1-e1*math.sin(alpha)**2 )
    M = a*(alpha*(1-e1/4-3*e1**2/64-5*e1**3/256)-math.sin(2*alpha)*(3*e1/8+3*e1**2/32+45*e1**3/1024)
        +math.sin(4*alpha)*(15*e1**2/256+45*e1**3/1024)-35*e1**3*math.sin(6*alpha)/3072)

    M0 = a*(alpha0*(1-e1/4-3*e1**2/64-5*e1**3/256)-math.sin(2*alpha0)*(3*e1/8+3*e1**2/32+45*e1**3/1024)
         +math.sin(4*alpha0)*(15*e1**2/256+45*e1**3/1024)-35*e1**3*math.sin(6*alpha0)/3072)

    Y = y_plus + k*N*(AA+AA**3*(1-T+C)/6+pow(AA,5)*(5-18*T+T*T+72*C-58*e2)/120)
    X = x_plus + k*(M-M0+N*math.tan(alpha)*(AA*AA/2 + pow(AA,4)*(5-T+9*C+4*C*C)/24 +
        pow(AA,6)*(61-58*T+T*T+600*C-330*e2)/720))

    return [X,Y]


def gps_callback(data):
    global lat, lon

    #print(odom_x)
    lat = data.latitude
    lon = data.longitude


def pub_global_ekf(lat,lon):
    x=[0,0]
    x = convert_degree_to_meter(lat,lon)
    #print(x)
    odommsg.header.stamp = rospy.Time.now()
    odommsg.header.frame_id = "base_footprint"
    odommsg.pose.pose.position.x = lat
    odommsg.pose.pose.position.y = lon
    odommsg.pose.pose.position.z = 0
    odommsg.pose.pose.orientation.x = 1
    odommsg.pose.pose.orientation.y = 0
    odommsg.pose.pose.orientation.z = 0
    odommsg.pose.pose.orientation.w = 0
    odommsg.pose.covariance = [1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 99999, 0, 0, 0, 0, 0, 0, 99999, 0, 0, 0, 0, 0, 0, 99999]
    #msg.pose.covariance = {cox_x, 0, 0, 0, 0, 0, 0, cov_y, 0, 0, 0, 0, 0, 0, cov_z, 0, 0, 0, 0, 0, 0, 99999, 0, 0, 0, 0, 0, 0, 99999, 0, 0, 0, 0, 0, 0, 99999}

    robot_pose_pub.publish(odommsg)
    #robot_pose_pub2.publish(odommsg)



if __name__ == '__main__':
    rospy.init_node("gps_node")

    rospy.Subscriber('/gps/fix', NavSatFix,gps_callback)

    robot_pose_pub=rospy.Publisher('vo', Odometry, queue_size=10)
    #robot_pose_pub2=rospy.Publisher('pr2_base_odometry/odom', Odometry, queue_size=10)
    gpsmsg=NavSatFix()
    odommsg=Odometry()

    lat =0
    lon =0
    rospy.loginfo("initialised")

    r=rospy.Rate(1)

    #yaw_sub = rospy.Subscriber("yaw_imu",Quaternion,cb_imu)


    while not rospy.is_shutdown():
        pub_global_ekf(lat,lon)

        #mainloop()
