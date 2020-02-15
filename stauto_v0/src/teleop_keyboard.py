#!/usr/bin/python

# This is a verison of turtlebot_teleop.py
# Copyright (c) 2020, stauto choiyungsik


import sys, select, termios, tty, math
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped

header_msg = """
Control stauto with teleop!
-------------------------
Moving around:
        i     
   j    k    l
        ,     
w/x : increase/decrease throttle bounds by 10%
a/d : increase/decrease steering bounds by 10%
s   : safety mode
space key, k : force stop
anything else : keep previous commands
CTRL-C to quit
"""

# Func for getting keyboard value
def getKey(safety_mode):
    if safety_mode: # wait unit keyboard interrupt
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    else: # pass if not detected
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

# Func for showing current bounds 
def showInfo(speed_bound, angle_bound):
    return "current bounds:\tspeed %s\tangle %s " % (speed_bound, angle_bound)

# Main Func
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('stauto_teleop')
    pub_cmd = rospy.Publisher('cmd_vel', TwistStamped, queue_size=5)
    pub_safe = rospy.Publisher('cmd_vel_safe', TwistStamped, queue_size=5)
    pub_pose = rospy.Publisher('STauto_base/pose',PoseStamped,queue_size=5)
    safe_mode = bool(rospy.get_param('~safety_mode', False)) # true for safety cmds (private_name) 
    speed_i = float(rospy.get_param('~speed_incremental', 0.1)) # m/s
    angle_i = float(rospy.get_param('~angle_incremental', 5.0*math.pi/180.0)) # rad (=5 degree)
    speed_bound = float(rospy.get_param('~speed_bound', 15.0))
    angle_bound = float(rospy.get_param('~angle_bound', 28.0*math.pi/180.0))
    
    if safe_mode:
        print "Switched to Safety Mode !!"

    moveBindings = {
            'i':(speed_i,0.0),
            'j':(0.0,angle_i),
            'l':(0.0,-angle_i),
            ',':(-speed_i,0.0),
               }

    boundBindings={
            'w':(1.1,1),
            'x':(.9,1),
            'a':(1,1.1),
            'd':(1,.9),
              }

    status = 0
    acc = 0.1
    target_speed = 0.0 # m/s
    target_angle = 0.0 # rad
    # Create TwistStamped msg object
    twist_msg = TwistStamped()
    pose_msg = PoseStamped()
    #twist_msg.header.frame_id = 'car_id' # for future multi-cars applicaton 

    try:
        print(header_msg)
        print(showInfo(speed_bound, angle_bound))
        while(1):
            key = getKey(safe_mode)
            if key in moveBindings.keys():
                target_speed = target_speed + moveBindings[key][0]
                target_angle = target_angle + moveBindings[key][1]
            elif key in boundBindings.keys():
                speed_bound = speed_bound * boundBindings[key][0]
                angle_bound = angle_bound * boundBindings[key][1]
                print(showInfo(speed_bound, angle_bound))
                if (status == 14):
                    print(header_msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                target_speed = 0.0
                target_angle = 0.0
            elif key == 's' : # switch safety mode
                safe_mode = not safe_mode
                if safe_mode:
                    print "Switched to Safety Mode !"
                else:
                    print "Back to Standard Mode !"
            elif key == '\x03': # cltr + C
                break

            # Command constraints
            if target_speed > speed_bound:
                target_speed = speed_bound
            if target_speed < -speed_bound:
                target_speed = -speed_bound
            if target_angle > angle_bound:
                target_angle = angle_bound
            if target_angle < -angle_bound:
                target_angle = -angle_bound

            # Publishing command
            #twist_msg.header.stamp = rospy.Time.now() # for future multi-cars applicaton 
            twist_msg.twist.linear.x = target_speed
            twist_msg.twist.angular.z = target_angle

            if safe_mode:
                pub_safe.publish(twist_msg)
            else:
                pub_cmd.publish(twist_msg)

    except Exception as e:
        print(e)

    finally:
        twist_msg.twist.linear.x = 0
        twist_msg.twist.angular.z = 0
        pub_cmd.publish(twist_msg)
        pub_safe.publish(twist_msg)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)