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

def speed_callback(data):
    global speed

    speed = data.data/10

def imu_callback(data):
    global imu_theta
    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    roll, pitch, yaw = euler_from_quaternion (orientation_list)

    imu_theta=yaw*(180/np.pi)
    #print(yaw)
def calculate_next_step(gps_n, gps_n_3, fp1, ld, utm_gps_cur):
    min_line=100
    utm_next_gps=[0,0]
    #print(gps_n, gps_n_3)
    #for i in range(gps_n, gps_n_3, 0.3):
    for i in np.arange(gps_n, gps_n_3, 0.2):

        goal_y= fp1[0]*i**(2)+fp1[1]*i+fp1[2]
        #goal_y= fp1[0]*i+fp1[1]
        #print(goal_y)
        line=sqrt((goal_y-utm_gps_cur[1])**(2) + (i-utm_gps_cur[0])**(2))

        if (line<min_line):
            min_line=line
            utm_next_gps=[i, goal_y]

    #print(utm_next_gps)
    return utm_next_gps

def find_gps_step():
    global last_step
    min_length=100
    cur_step=0
    for step_gps in range(last_step-4):
        gps_n = gps_data[step_gps].split(',')
        gps_n_1 = gps_data[step_gps+1].split(',')
        gps_n_2 = gps_data[step_gps+2].split(',')


        # gps_n = [float(gps_n[0]) - float(gps_origin[0]), float(gps_n[1]) - float(gps_origin[1])]
        # gps_n_1 = [float(gps_n_1[0]) - float(gps_origin[0]), float(gps_n_1[1])- float(gps_origin[1])]
        gps_n = [float(gps_n[0]), float(gps_n[1])]
        gps_n_1 = [float(gps_n_1[0]), float(gps_n_1[1])]
        gps_n_2 = [float(gps_n_2[0]), float(gps_n_2[1])]


        utm_gps_n = convert_degree_to_meter(gps_n[0],gps_n[1])
        utm_gps_n_1 = convert_degree_to_meter(gps_n_1[0],gps_n_1[1])
        utm_gps_n_2 = convert_degree_to_meter(gps_n_2[0],gps_n_2[1])
        utm_gps_cur = convert_degree_to_meter(lat,lon)

        length1 = sqrt((utm_gps_cur[0]-utm_gps_n[0])**(2)+(utm_gps_cur[1]-utm_gps_n[1])**(2))
        length2 = sqrt((utm_gps_cur[0]-utm_gps_n_1[0])**(2)+(utm_gps_cur[1]-utm_gps_n_1[1])**(2))

        length = length1+length2
        '''
        line_data_x=[utm_gps_n[0],utm_gps_n_1[0],utm_gps_n_2[0]]
        line_data_y=[utm_gps_n[1],utm_gps_n_1[1],utm_gps_n_2[1]]

        fp1 = np.polyfit(line_data_x,line_data_y,1)
        
    
        y= fp1[0]*step_gps+fp1[1]
        
        length=abs(fp1[0]*utm_gps_cur[0] - utm_gps_cur[1] + fp1[1])/sqrt(fp1[0]**(2)+(-1)**(2)) #find length
        '''

        if(length<min_length):
            min_length=length
            cur_step=step_gps+1

    return cur_step

if __name__ == '__main__':

    rospy.init_node('gps_path')
    listener = tf.TransformListener()

    rospy.Subscriber("/gps/fix",NavSatFix,gps_callback)
    rospy.Subscriber("/imu/data",Imu,imu_callback)
    rospy.Subscriber("/ERP42_speed",Float32,speed_callback)

    path_pub = rospy.Publisher('/path', Point, queue_size=10)
    start_yaw = rospy.Publisher('yaw_error', Float32, queue_size=10)
    pure_pursuit_pub = rospy.Publisher('/pure_pursuit', Float32, queue_size=10)
    #goal_pub = rospy.Publisher('/pure_pursuit/goal', PoseStamped, queue_size=1)
    #navs_pub = rospy.Publisher('/fix', NavSatFix, queue_size=1)

    rospack = rospkg.RosPack()
    rospack.list()
    arg_name = rospack.get_path('stauto_sensor') + "/src/gps_data/"

    f = open(arg_name + "gps_data_seoultech_fron_rotary.txt","r")

    gps_data = f.readlines()
    last_step=len(gps_data)
    imu_theta=0.
    lat=0
    lon=0
    step_theta=0
    error_yaw=0
    alpha=0
    delta=0
    speed=0
    prev_ld_position=[0,0]
    start_yaw_sign=True
    '''
    utm_gps_n=[0,0]
    utm_gps_n_1=[0,0]
    utm_gps_n_2=[0,0]
    utm_gps_n_3=[0,0]
    utm_gps_cur=[0,0]
    utm_next_gps=[0,0]
    '''

    going_gps_n=[0,0]
    going_gps_n1=[0,0]
    going_gps_n3=[0,0]
    going_gps=[0,0]


    theta_gps_n=[0,0,0]
    
    gpsmsg=NavSatFix()
    data=Point()

    rospy.sleep(1)
    
    init_time=rospy.Time.now()
    step_gps=find_gps_step()
    while not rospy.is_shutdown():
        #s(_,rot) = listener.lookupTransform('pose', 'base_link', rospy.Time(0))
        #yaw = -tf.transformations.euler_from_quaternion(rot)[2]

        if (step_gps<last_step-4):

            #print(find_gps_step())
            gps_n = gps_data[step_gps].split(',')
            gps_n_1 = gps_data[step_gps+1].split(',')
            gps_n_2 = gps_data[step_gps+2].split(',')
            gps_n_3 = gps_data[step_gps+3].split(',')

            # gps_n = [float(gps_n[0]) - float(gps_origin[0]), float(gps_n[1]) - float(gps_origin[1])]
            # gps_n_1 = [float(gps_n_1[0]) - float(gps_origin[0]), float(gps_n_1[1])- float(gps_origin[1])]
            gps_n = [float(gps_n[0]), float(gps_n[1])]
            gps_n_1 = [float(gps_n_1[0]), float(gps_n_1[1])]
            gps_n_2 = [float(gps_n_2[0]), float(gps_n_2[1])]
            gps_n_3 = [float(gps_n_3[0]), float(gps_n_3[1])]


            utm_gps_n = convert_degree_to_meter(gps_n[0],gps_n[1])
            utm_gps_n_1 = convert_degree_to_meter(gps_n_1[0],gps_n_1[1])
            utm_gps_n_2 = convert_degree_to_meter(gps_n_2[0],gps_n_2[1])
            utm_gps_n_3 = convert_degree_to_meter(gps_n_3[0],gps_n_3[1])
            utm_gps_cur = convert_degree_to_meter(lat,lon)

            line_data_x=[utm_gps_n[0],utm_gps_n_1[0],utm_gps_n_2[0],utm_gps_n_3[0]]
            line_data_y=[utm_gps_n[1],utm_gps_n_1[1],utm_gps_n_2[1],utm_gps_n_3[1]]
            '''
            fp1 = np.polyfit(line_data_x,line_data_y,2)

            ld = speed*0.237+2.5
            #print(speed,ld)
            utm_next_gps = calculate_next_step(utm_gps_n[0], utm_gps_n_3[0], fp1, ld, utm_gps_cur)

            going_gps[0]=(utm_gps_cur[0] - 460000)/100
            going_gps[1]=(utm_gps_cur[1] - 383000)/100

            #going_gps_n[0]=(utm_next_gps[0] - 460000)/100
            #going_gps_n[1]=(utm_next_gps[1] - 383000)/100

            #test
            going_gps_n[0]=(utm_gps_n_2[0] - 460000)/100
            going_gps_n[1]=(utm_gps_n_2[1] - 383000)/100

            going_gps_n1[0]=(utm_gps_n_1[0] - 460000)/100
            going_gps_n1[1]=(utm_gps_n_1[1] - 383000)/100
            going_gps_n3[0]=(utm_gps_n_3[0] - 460000)/100
            going_gps_n3[1]=(utm_gps_n_3[1] - 383000)/100

            rospy.sleep(0.1)

            data.x=0
            data.y=gps_n_2[0]
            data.z=gps_n_2[1]

            path_pub.publish(data)

            #print(going_gps_n[0], going_gps[0],going_gps_n[1],going_gps[1])
            going_gps_theta = atan2(going_gps_n[0]-going_gps[0], going_gps_n[1]-going_gps[1])*180/np.pi

            if (step_gps>=0) and (start_yaw_sign==True):
                start_theta = atan2(going_gps_n3[0]-going_gps_n1[0], going_gps_n3[1]-going_gps_n1[1])*180/np.pi
                print(start_theta)
                start_yaw.publish(-start_theta)
                start_yaw_sign=False

            #print(going_gps_theta, imu_theta)
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

            Ld = sqrt((utm_gps_n_1[1]-utm_gps_cur[1])**(2) + (utm_gps_n_1[0]-utm_gps_cur[0])**(2))
            L=1.3

            speed_ld = speed*0.237
            alpha_ld = abs(alpha*180/np.pi)*0.05

            if(alpha_ld>=2):
                alpha_ld=2

            ld = speed*0.237+3-alpha_ld
            #print(speed_ld,alpha_ld, ld)
            error_yaw = 0
            delta=atan(2*L*sin(alpha)/ld)*(180/np.pi)+error_yaw
            pure_pursuit_pub.publish(delta)
            print(Ld, step_gps, delta)
            
            if(abs(Ld) <=2.5)and(step_gps<=last_step-4):
                step_gps=step_gps+1


            #rospy.sleep(1)
            #Line = sqrt((gps_n[0]-trans[0])**2 + (gps_n[1]-trans[1])**2)
        else:
            pass

