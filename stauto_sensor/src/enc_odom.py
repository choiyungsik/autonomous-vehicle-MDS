#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
<<<<<<< HEAD
from std_msgs.msg import Int32
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def enc_callback(data):
    global curl_encoder, current_time

    curl_encoder=data.data

def steer_callback(data):
    global steer

    steer=data.data

def calculation(steer,dt):
    global curl_encoder, last_encoder
    
    if(curl_encoder==last_encoder):
        speed=0
    if(curl_encoder > last_encoder):
        try:
            move_encoder = curl_encoder-last_encoder
            last_encoder = curl_encoder
            rad = move_encoder*0.06283185307 # 3.6[deg] * 3.14159265359 / 180 = 0.06283185307, encoder 100 -> 1 rev
            wheel_vth = rad/dt
            speed = wheel_vth * 0.265 #wheel_radius
            #print(move_encoder, speed)
=======
from std_msgs.msg import Int32, Float32, Float64, Header
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

cur_encoder = 0     # Encoder counting
prev_encoder = 0
steer = 0           # -2000 ~ 2000 (actual steering angle * 71)
speed = 0           # 0~200(actual speed(KPH)*10) 

def EncCallback(data):
    global cur_encoder, current_time

    cur_encoder=data.data

def SteerCallback(data):
    global steer

    steer= (data.data) * 0.014 #150 for align

def SpeedCallback(data):
    global erp_speed

    erp_speed = data.data/72


########### odometry calculation ###############

def calculation(steer,dt,erp_speed_):
    global cur_encoder, prev_encoder
    
    if(cur_encoder == prev_encoder):
        speed=0

    elif(cur_encoder > prev_encoder):       # going backward
        try:
            diff_encoder = abs(cur_encoder-prev_encoder)
            prev_encoder = cur_encoder
            rad = diff_encoder*0.06283185307 # 3.6[deg] * 3.14159265359 / 180 = 0.06283185307, encoder 100 -> 1 rev

            wheel_vth = rad/dt
            speed = wheel_vth * 0.265 #wheel_radius
            print(diff_encoder, speed)
>>>>>>> 681e6b4e10842f5eb63e31c1372ba9d7d1a8f83f

        except ZeroDivisionError:
            speed=0

<<<<<<< HEAD

    elif(curl_encoder < last_encoder):
        try:
            move_encoder = 255 + (curl_encoder-last_encoder)
            last_encoder = curl_encoder
            rad = move_encoder*0.06283185307 # 3.6[deg] * 3.14159265359 / 180 = 0.06283185307, encoder 100 -> 1 rev
=======
    elif(cur_encoder < prev_encoder):       #going forward
        try:
            diff_encoder = abs(cur_encoder-prev_encoder)
            prev_encoder = cur_encoder
            rad = diff_encoder*0.06283185307 # 3.6[deg] * 3.14159265359 / 180 = 0.06283185307, encoder 100 -> 1 rev
>>>>>>> 681e6b4e10842f5eb63e31c1372ba9d7d1a8f83f

            wheel_vth = rad/dt
            speed = wheel_vth * 0.265 #wheel_radius

        except ZeroDivisionError:
            speed=0

<<<<<<< HEAD
    else:
        move_encoder = 0

        speed = 0

    #print(speed)
    vx = speed
    vy = 0
    #print(steer, speed)
    steer = steer*(pi/180)
    #print(math.tan((pi/2)+steer))
    #print(steer)

    if(steer < 0.):
        try:
            r = 1.03*math.tan((pi/2)+steer)
            vth = speed/r
=======
    speed = erp_speed_
    vx = speed
    vy = 0
    steer = steer*(pi/180)


    if(steer < 0.):
        try:
            r = 1.03/math.tan(steer)
            vth = speed* 0.6/r #0.6 parameter
>>>>>>> 681e6b4e10842f5eb63e31c1372ba9d7d1a8f83f

            return vx, vy, vth
        except ZeroDivisionError:
            vth = 0

            return vx, vy, vth


    elif(steer > 0.):
        try:
<<<<<<< HEAD
            r = -1.03*math.tan((pi/2)-steer)
            vth = speed/r
            print(r)
=======
            r = 1.03/math.tan(steer)
            vth = speed* 0.72/r #0.72 parameter
            # print(r)
>>>>>>> 681e6b4e10842f5eb63e31c1372ba9d7d1a8f83f
            return vx, vy, vth
        except ZeroDivisionError:
            vth = 0

            return vx, vy, vth

    else:
        vth = 0

        return vx, vy, vth



if __name__ == '__main__':
    rospy.init_node('odometry_publisher')

<<<<<<< HEAD
    rospy.Subscriber("ENCODER_DATA",Int32,enc_callback)
    rospy.Subscriber("STEER",Int32,steer_callback)
=======
    rospy.Subscriber("ERP42_encoder",Float64,EncCallback) # 4 bytes
    rospy.Subscriber("ERP42_steer",Float32,SteerCallback) # 2 bytes
    rospy.Subscriber("ERP42_speed",Float32,SpeedCallback) # 2 bytes
>>>>>>> 681e6b4e10842f5eb63e31c1372ba9d7d1a8f83f

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

<<<<<<< HEAD

    curl_encoder=0
    steer=0
    last_encoder=0
=======
    erp_speed = 0
    cur_encoder=0
    steer=0
    prev_encoder=0
>>>>>>> 681e6b4e10842f5eb63e31c1372ba9d7d1a8f83f

    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0
    vy = 0
    vth = 0

    dt = 0

<<<<<<< HEAD
    last_encoder = curl_encoder
=======
    prev_encoder = cur_encoder
>>>>>>> 681e6b4e10842f5eb63e31c1372ba9d7d1a8f83f


    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    r = rospy.Rate(10)


    while not rospy.is_shutdown():


        # compute odometry in a typical way given the velocities of the robot

        #last_time = current_time
        #rospy.spinOnce()
        current_time = rospy.Time.now()

<<<<<<< HEAD
        dt = (current_time - last_time).to_sec()
        if dt>0:
            vx, vy, vth = calculation(steer,dt)
=======
        dt = (current_time - last_time).to_sec() + (current_time - last_time).to_nsec()*1e-9
        if dt>0:
            vx, vy, vth = calculation(steer,dt,erp_speed)
>>>>>>> 681e6b4e10842f5eb63e31c1372ba9d7d1a8f83f
        delta_x = (vx * cos(th) - vy * sin(th)) * dt
        delta_y = (vx * sin(th) + vy * cos(th)) * dt
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th
<<<<<<< HEAD
        #print(th)
=======

>>>>>>> 681e6b4e10842f5eb63e31c1372ba9d7d1a8f83f
        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
<<<<<<< HEAD
            "base_link",
=======
            "base_footprint",
>>>>>>> 681e6b4e10842f5eb63e31c1372ba9d7d1a8f83f
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
<<<<<<< HEAD
        odom.child_frame_id = "base_link"
=======
        odom.child_frame_id = "base_footprint"
>>>>>>> 681e6b4e10842f5eb63e31c1372ba9d7d1a8f83f
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        odom_pub.publish(odom)
<<<<<<< HEAD
        #print(odom.pose.pose)

        last_time = current_time
        r.sleep
=======

        last_time = current_time
        r.sleep()
>>>>>>> 681e6b4e10842f5eb63e31c1372ba9d7d1a8f83f
