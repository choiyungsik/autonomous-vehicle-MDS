#! /usr/bin/env python

import rospy
import tf
import numpy as np
import rospkg
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
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

def gps_callback(data):
    global lat, lon

    #print(odom_x)
    lat = data.latitude
    lon = data.longitude

def imu_callback(data):
    global imu_theta
    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    roll, pitch, yaw = euler_from_quaternion (orientation_list)

    imu_theta=yaw*(180/np.pi)
    #print(yaw)

if __name__ == '__main__':

    rospy.init_node('gps_path')
    listener = tf.TransformListener()

    rospy.Subscriber("/gps/fix",NavSatFix,gps_callback)
    rospy.Subscriber("/imu/data",Imu,imu_callback)

    #navs_pub = rospy.Publisher('/fix', NavSatFix, queue_size=1)
    path_pub = rospy.Publisher('/path', Point, queue_size=10)
    pure_pursuit_pub = rospy.Publisher('/pure_pursuit', Float32, queue_size=10)
    #goal_pub = rospy.Publisher('/pure_pursuit/goal', PoseStamped, queue_size=1)

    rospack = rospkg.RosPack()
    rospack.list()
    arg_name = rospack.get_path('stauto_sensor') + "/src/gps_data/"

    f = open(arg_name + "gps_data_seoultech.txt","r")

    gps_data = f.readlines()
    step_gps=0
    imu_theta=0.
    lat=0
    lon=0
    step_theta=0
    error_yaw=0
    alpha=0
    delta=0
    gps_nn=[0,0]
    gps_nn_1=[0,0]
    prize_gps_nn=[0,0]
    theta_gps_n=[0,0,0]
    last_step=len(gps_data)
    gpsmsg=NavSatFix()
    data=Point()

    rospy.sleep(1)

    init_time=rospy.Time.now()

    while not rospy.is_shutdown():
        #s(_,rot) = listener.lookupTransform('pose', 'base_link', rospy.Time(0))
        #yaw = -tf.transformations.euler_from_quaternion(rot)[2]

        if (step_gps<last_step-4):

            '''
            gps_n = gps_data[step_gps].split(',')
            gps_n_1 = gps_data[step_gps+1].split(',')
            gps_n_2 = gps_data[step_gps+2].split(',')
            gps_n_3 = gps_data[step_gps+3].split(',')
            gps_n_4 = gps_data[step_gps+4].split(',')
            # gps_n = [float(gps_n[0]) - float(gps_origin[0]), float(gps_n[1]) - float(gps_origin[1])]
            # gps_n_1 = [float(gps_n_1[0]) - float(gps_origin[0]), float(gps_n_1[1])- float(gps_origin[1])]
            gps_n = [float(gps_n[0]), float(gps_n[1])]
            gps_n_1 = [float(gps_n_1[0]), float(gps_n_1[1])]
            gps_n_2 = [float(gps_n_2[0]), float(gps_n_2[1])]
            gps_n_3 = [float(gps_n_3[0]), float(gps_n_3[1])]
            gps_n_4 = [float(gps_n_4[0]), float(gps_n_4[1])]

            line_data_x=[gps_n[0],gps_n_1[0],gps_n_2[0],gps_n_4[0],gps_n_4[0]]
            line_data_y=[gps_n[1],gps_n_1[1],gps_n_2[1],gps_n_3[1],gps_n_4[1]]

            fp1 = np.polyfit(line_data_x,line_data_y,2)

            goal_x=(gps_n_3[0]+gps_n_2[0])/2 #pure_pursuit point
            goal_y= fp1[0]*goal_x**(2)+fp1[1]*goal_x+fp1[2]
            '''

            gps_n = gps_data[step_gps].split(',')
            gps_n_1 = gps_data[step_gps+1].split(',')
            gps_n_2 = gps_data[step_gps+2].split(',')

            # gps_n = [float(gps_n[0]) - float(gps_origin[0]), float(gps_n[1]) - float(gps_origin[1])]
            # gps_n_1 = [float(gps_n_1[0]) - float(gps_origin[0]), float(gps_n_1[1])- float(gps_origin[1])]
            gps_n = [float(gps_n[0]), float(gps_n[1])]
            gps_n_1 = [float(gps_n_1[0]), float(gps_n_1[1])]
            gps_n_2 = [float(gps_n_2[0]), float(gps_n_2[1])]

            #print(lat,lon, gps_n)
            #line_data_x=[gps_n[0],gps_n_1[0],gps_n_2[0]]
            #line_data_y=[gps_n[1],gps_n_1[1],gps_n_2[1]]
            prev_step = convert_degree_to_meter(gps_n[0],gps_n[1])
            next_step = convert_degree_to_meter(gps_n_1[0],gps_n_1[1])
            #print(gps_n_1[1])
            gps_nn[0] = (prev_step[0] - 460000)/100
            gps_nn[1] = (prev_step[1] - 383000)/100
            gps_nn_1[0] = (next_step[0] - 460000)/100
            gps_nn_1[1] = (next_step[1] - 383000)/100

            #fp1 = np.polyfit(line_data_x,line_data_y,2)

            #goal_x=(gps_n_2[0]+gps_n_1[0])/2 #pure_pursuit point
            #goal_y= fp1[0]*goal_x**(2)+fp1[1]*goal_x+fp1[2]

            step_theta = atan2(gps_nn_1[0]-gps_nn[0], gps_nn_1[1]-gps_nn[1])*180/np.pi
            #print("fp:", fp1,"theta", theta)

            rospy.sleep(0.1)

            data.x=step_theta
            data.y=gps_n_2[0]
            data.z=gps_n_2[1]

            path_pub.publish(data)
            #print(lat,lon)
            gps_n = convert_degree_to_meter(lat,lon)

            gps_n_1 = convert_degree_to_meter(gps_n_1[0],gps_n_1[1])
            #print(lat, lon, gps_n_2[0], gps_n_2[1])

            prize_gps_nn[0] = (gps_n[0] - 460000)/100
            prize_gps_nn[1] = (gps_n[1] - 383000)/100

            prize_theta = atan2(gps_nn_1[0]-prize_gps_nn[0], gps_nn_1[1]-prize_gps_nn[1])*180/np.pi
            #print(prize_theta, theta)
            if((int(imu_theta)*int(prize_theta))>=0.):
                alpha=imu_theta-prize_theta
            else:
                if(((-90>=imu_theta>=-180) or (180>=imu_theta>=90)) and ((-90>=prize_theta>=-180) or (180>=prize_theta>=90))):
                    if((imu_theta>=0) and (prize_theta<0)):
                        alpha=-((180-imu_theta)+(180+prize_theta))
                    elif((imu_theta<0) and (prize_theta>=0)):
                        alpha=(180+imu_theta)+(180-prize_theta)
                else:
                    alpha=imu_theta-prize_theta

            alpha=alpha*(np.pi/180)
            #print(gps_n_1[1], gps_n[1])
            Ld = sqrt((gps_n_1[1]-gps_n[1])**(2) + (gps_n_1[0]-gps_n[0])**(2))
            L = 1.3

            error_yaw = 2
            delta=atan(2*L*sin(alpha)/Ld)*(180/np.pi)+error_yaw
            pure_pursuit_pub.publish(delta)
            print(Ld, step_gps, delta)
            if(abs(Ld) <=2.5)and(step_gps<=last_step-4):
                step_gps=step_gps+1


            #rospy.sleep(1)
            #Line = sqrt((gps_n[0]-trans[0])**2 + (gps_n[1]-trans[1])**2)
        else:
            pass

#!/usr/bin/env python
