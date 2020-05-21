#!/usr/bin/python

import sys, select, termios, tty, math
from math import cos, sin, pi, tan, atan, atan2
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped, PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

header_msg = """
Control stauto_car!
-------------------------
Moving around:
        i     
   j    k    l
        ,     
w/x : increase/decrease throttle bounds by 10%
e/c : increase/decrease steering bounds by 10%
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

def cal_odometry(xpos,ypos,yaw,xvel,yvel):
    odom_msg = Odometry()

    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"


    odom_msg.pose.pose.position.x = xpos
    odom_msg.pose.pose.position.y = ypos
    odom_msg.pose.pose.position.z = 0.0

    odom_msg.pose.pose.orientation = tf.transformations.quaternion_from_euler(0,0,yaw)

    odom_msg.twist.twist.linear.x = xvel
    odom_msg.twist.twist.linear.y = yvel
    odom_msg.twist.twist.angular.z = 0.0

    return odom_msg


class initpose:
    def __init__(self):
        self.pre_position = [0,0,0]
        self.position = [0,0,0]
        self.orientation_quaternion = [0,0,0,0]
        
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,self.initialCallback)

    def initialCallback(self,data):
        pose_stemp = data
        self.position = [pose_stemp.pose.pose.position.x, pose_stemp.pose.pose.position.y, pose_stemp.pose.pose.position.z]
        self.orientation_quaternion = [pose_stemp.pose.pose.orientation.x, pose_stemp.pose.pose.orientation.y, pose_stemp.pose.pose.orientation.z, pose_stemp.pose.pose.orientation.w]

        print("init")


# Main Func
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('stauto_teleop')
    odom_broadcaster = tf.TransformBroadcaster()
    
    #pub_cmd = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=5)
    #pub_safe = rospy.Publisher('/ackermann_safe', AckermannDriveStamped, queue_size=5)
    pub_odom = rospy.Publisher('/odom_tele',Odometry)

    safe_mode = bool(rospy.get_param('~safety_mode', False)) # true for safety cmds 
    speed_i = float(rospy.get_param('~speed_incremental', 0.1)) # m/s
    angle_i = float(rospy.get_param('~angle_incremental', 5.0*math.pi/180.0)) # rad (=5 degree)
    speed_bound = float(rospy.get_param('~speed_bound', 2.0))
    angle_bound = float(rospy.get_param('~angle_bound', 20.0*math.pi/180.0))

    x = 0.0
    y = 0.0
    th = 0.0
    tau = 0.0
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    class_init_pose = initpose()

    #setting map and odom tf
    
    
    if safe_mode:
        print ("Switched to Safety Mode !")

    moveBindings = {
            'i':(speed_i,0.0),
            'j':(0.0,angle_i),
            'l':(0.0,-angle_i),
            ',':(-speed_i,0.0),
               }

    boundBindings={
            'w':(1.1,1),
            'x':(.9,1),
            'e':(1,1.1),
            'c':(1,.9),
              }

    status = 0
    acc = 0.1
    target_speed = 0.0 # m/s
    target_angle = 0.0 # rad
    # Create AckermannDriveStamped msg object
    ackermann_msg = AckermannDriveStamped()
    odom = Odometry()

    try:
        print(header_msg)
        print(showInfo(speed_bound, angle_bound))
        while(1):
            current_time = rospy.Time.now()

            dt= (current_time - last_time).to_sec()

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
                    print ("Switched to Safety Mode !")
                else:
                    print ("Back to Standard Mode !")
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

            beta = atan((0.7/1.4)*tan(target_angle))

            delta_tau = (target_speed/0.7)*sin(beta)
            tau += delta_tau

            delta_x = (target_speed * cos(tau + beta)) * dt
            delta_y = (target_speed * sin(tau + beta)) * dt

            # Publishing command
            #ackermann_msg.header.stamp = rospy.Time.now() # for future multi-cars applicaton
            x += delta_x
            y += delta_y

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, tau)

            odom_broadcaster.sendTransform(
                class_init_pose.position,
                class_init_pose.orientation_quaternion,
                current_time,
                "odom",
                "map"
            )
            odom_broadcaster.sendTransform(
                (x, y, 0.2),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )
            odom = cal_odometry(x,y,tau,target_speed * cos(tau + beta),target_speed * sin(tau + beta))
            pub_odom.publish(odom)

            last_time = current_time

    except Exception as e:
        print(e)

    finally:
        ackermann_msg.drive.speed = 0
        ackermann_msg.drive.steering_angle = 0

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)