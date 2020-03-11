#! /usr/bin/env python

import serial
import socket as soc
import rospkg
import rospy

import time
import rospkg
from sensor_msgs.msg import NavSatFix
import sys, select, termios, tty

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def gps_callback(data):

    lat=data.latitude
    lon=data.longitude

    print('good')
    f.write(str(lat))
    f.write(','+str(lon)+'\n')


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)


    rospy.init_node('save_gps')
    rospy.Subscriber("/gps/fix", NavSatFix,gps_callback)

    rospack = rospkg.RosPack()
    rospack.list()
    arg_name = rospack.get_path('stauto_sensor') + "/src/gps_data/"
    f=open(arg_name + "gps_data_seoultech.txt",'w')
    try:
        while not rospy.is_shutdown():
            key=getKey()


            if key=='esc':
                print("save ok")
                f.close()
                break

    except rospy.ROSInterruptException:
        pass

            #print ("Missed" ,"\r")
        #mainloop()
