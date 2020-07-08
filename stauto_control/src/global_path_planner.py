#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def global_path_callback(data):
    global path, global_path_callback_sign
    global_path_callback_sign=True
    path=data

def current_step_callback(data):
    global step

    step=int(data.pose.position.z)

def pub_path(path, step):
    pathmsg=Path()

    pathmsg.header.seq = step
    pathmsg.header.stamp = rospy.Time.now()
    pathmsg.header.frame_id = "map"

    for i in range(4):
        pose = PoseStamped()

        pose.header.seq = step
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        print(step)
        pose.pose.position.x = path.poses[step+i].pose.position.x
        pose.pose.position.y = path.poses[step+i].pose.position.y
        pose.pose.position.z = 0

        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 0

        #rospy.sleep(0.5)
        pathmsg.poses.append(pose)

    global_path.publish(pathmsg)

if __name__ == '__main__':
    rospy.init_node('path_planner')

    rospy.Subscriber("global_path",Path,global_path_callback)
    rospy.Subscriber("current_step",PoseStamped,current_step_callback)

    global_path = rospy.Publisher("/local_path",Path, queue_size=10)

    rate = rospy.Rate(10)

    step=0
    global_path_callback_sign=False
    path=Path()
    #rospy.sleep(10)
    while not rospy.is_shutdown():
        if (global_path_callback_sign==True):
            rospy.sleep(0.1)
            pub_path(path, step)
        else:
            pass
