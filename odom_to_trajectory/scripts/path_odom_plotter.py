#!/usr/bin/env python
from __future__ import print_function
import rospy
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from sensor_msgs.msg import Joy

import sys
import json
from math import sqrt
from collections import deque

import time


def callback(data):
        global xAnt
        global yAnt
        global cont

    #Is created the pose msg, its necessary do it each time because Python manages objects by reference, 
        #and does not make deep copies unless explicitly asked to do so.
        pose = PoseStamped()    

    #Set a atributes of the msg
        pose.header.frame_id = "base_link"
        pose.pose.position.x = float(data.pose.pose.position.x)
        pose.pose.position.y = float(data.pose.pose.position.y)
        pose.pose.orientation.x = float(data.pose.pose.orientation.x)
        pose.pose.orientation.y = float(data.pose.pose.orientation.y)
        pose.pose.orientation.z = float(data.pose.pose.orientation.z)
        pose.pose.orientation.w = float(data.pose.pose.orientation.w)

    #To avoid repeating the values, it is found that the received values are differents
        if (xAnt != pose.pose.position.x and yAnt != pose.pose.position.y):
                #Set a atributes of the msg
                pose.header.seq = path.header.seq + 1
                if(amcl_mode):
                    path.header.frame_id="map"
                else:
                    path.header.frame_id="odom"
                path.header.stamp=rospy.Time.now()
                pose.header.stamp = path.header.stamp
                path.poses.append(pose)
                #Published the msg

        cont=cont+1

        # rospy.loginfo("Valor del contador: %i" % cont)
        if cont>max_append:
        	path.poses.pop(0)

        pub.publish(path)

    #Save the last position
        xAnt=pose.pose.orientation.x
        yAnt=pose.pose.position.y
        return path

def callback_enc(data_enc):
        global xAnt_enc
        global yAnt_enc
        global cont_enc

    #Is created the pose msg, its necessary do it each time because Python manages objects by reference, 
        #and does not make deep copies unless explicitly asked to do so.
        pose_enc = PoseStamped()    

    #Set a atributes of the msg
        pose_enc.header.frame_id = "base_footprint"
        pose_enc.pose.position.x = float(data_enc.pose.pose.position.x)
        pose_enc.pose.position.y = float(data_enc.pose.pose.position.y)
        pose_enc.pose.orientation.x = float(data_enc.pose.pose.orientation.x)
        pose_enc.pose.orientation.y = float(data_enc.pose.pose.orientation.y)
        pose_enc.pose.orientation.z = float(data_enc.pose.pose.orientation.z)
        pose_enc.pose.orientation.w = float(data_enc.pose.pose.orientation.w)

    #To avoid repeating the values, it is found that the received values are differents
        if (xAnt_enc != pose_enc.pose.position.x and yAnt_enc != pose_enc.pose.position.y):
                #Set a atributes of the msg
                pose_enc.header.seq = path_enc.header.seq + 1
                if(amcl_mode):
                    path_enc.header.frame_id="start"
                else:
                    path_enc.header.frame_id="odom"
                path_enc.header.stamp=rospy.Time.now()
                pose_enc.header.stamp = path_enc.header.stamp
                path_enc.poses.append(pose_enc)
                #Published the msg

        cont_enc=cont_enc+1

        # rospy.loginfo("Valor del contador: %i" % cont_enc)
        if cont_enc>max_append:
        	path_enc.poses.pop(0)

        pub_enc.publish(path_enc)

    #Save the last position
        xAnt_enc=pose_enc.pose.orientation.x
        yAnt_enc=pose_enc.pose.position.y
        return path_enc


if __name__ == '__main__':
        #Variable initialization
        global xAnt
        global yAnt
        global cont
        global xAnt_enc
        global yAnt_enc
        global cont_enc
        xAnt=0.0
        yAnt=0.0
        cont=0
        xAnt_enc=0.0
        yAnt_enc=0.0
        cont_enc=0


        #Node and msg initialization
        rospy.init_node('path_odom_plotter')

        amcl_mode = bool(rospy.get_param('~use_amcl', False)) # 

        #Rosparams that are set in the launch
        #max size of array pose msg from the path
        # if not rospy.has_param("~max_list_append"):
        #         rospy.logwarn('The parameter max_list_append dont exists')
        # max_append = rospy.set_param("~max_list_append",10000000) 
        max_append = 10000000
        if not (max_append > 0):
                rospy.logwarn('The parameter max_list_append not is correct')
                sys.exit()
        pub = rospy.Publisher('/odompath', Path, queue_size=1)
        pub_enc = rospy.Publisher('/odompath/enc', Path, queue_size=1)


        path = Path() #creamos el mensaje path de tipo path 
        path_enc = Path()
        msg = Odometry()
        msg_enc = Odometry()

        #Subscription to the topic
        if(amcl_mode):
            msg = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback) 
        else:
            msg = rospy.Subscriber('/odom', Odometry, callback) 
        msg_enc = rospy.Subscriber('/odom/enc', Odometry, callback_enc) 

        rate = rospy.Rate(30) # 30hz

try:
	while not rospy.is_shutdown():
        	#rospy.spin()
        	rate.sleep()
except rospy.ROSInterruptException:
	pass
