#! /usr/bin/env python
import numpy as np
import rospy
import pynput
import time
import rospkg
from math import *

from geometry_msgs.msg import Twist
data1=Twist()

def cv_pos(data):
    global data1
    data1=data
    return data1#    trans[0] = trans[0] + offset_X
    #    trans[1] = trans[1] + offset_Y




if __name__ == '__main__':

    rospy.init_node('cmd_vel_test',disable_signals=True)
    #listener = tf.TransformListener()
    sub_pos = rospy.Subscriber("fake_cmd", Twist, cv_pos)

    rate = rospy.Rate(10)


    while not rospy.is_shutdown():
        try:
            a=data1

            print(a)



        except (KeyboardInterrupt, RuntimeError) as ident:
            print('error')
