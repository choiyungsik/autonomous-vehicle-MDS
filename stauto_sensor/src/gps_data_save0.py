#! /usr/bin/env python3
#import keyboard
import rospy
from sensor_msgs.msg import NavSatFix
import serial
import socket as soc
import rospkg
#import rospy
#import keyboard
import time
from ntrip.NtripClient import *

import multiprocessing
from multiprocessing import Process, Queue

import os
import sys

import math

#yaw = Quaternion()
#pos = PointStamped()
gps_data_bef = ""
prev_time=0


def gps_callback(data):
    global lat, lon

    #print(odom_x)
    lat = data.latitude
    lon = data.longitude

if __name__ == '__main__':
    rospy.init_node('gps_save')

    isrunning=True
    isReady=False
    count_ready = 0
    t=time.time()
    prev_time_rtk=0
    reRTK_count=True
    prev_time=0
    prev_pos = [0.0,0.0]

    lat=0
    lon=0
    rospy.Subscriber("/gps/fix",NavSatFix,gps_callback)
    rospack = rospkg.RosPack()
    rospack.list()
    arg_name = rospack.get_path('stauto_sensor') + "/src/gps_data/"
    f=open(arg_name+"gps_data_seoultech.txt",'w')
    while not rospy.is_shutdown():

        if (time.time()-prev_time>=1):
            print('good')
            f.write(str(lat))
            f.write(','+str(lon)+'\n')
            prev_time=time.time()
            #print('good!')


        #if keyboard.is_pressed('esc'):
        #    print('save ok')
        #    f.close()
        #    break

    else:
        pass
