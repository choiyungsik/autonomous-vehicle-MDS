#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool

MIN_NUM = float('-inf')
MAX_NUM = float('inf')

class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0

    def step(self, error, sample_time):

        integral = self.int_val + error * sample_time;
        derivative = (error - self.last_error) / sample_time;

        val = self.kp * error + self.ki * integral + self.kd * derivative;

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0

  radius = v / omega
  return math.atan(wheelbase / radius)


def cmd_callback(data):
  global wheelbase
  global ackermann_cmd_topic
  global frame_id
  global pub
  global msg
  global v

  v = data.linear.x
  steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
  
  msg = AckermannDriveStamped()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = frame_id
  msg.drive.steering_angle = steering
  # msg.drive.speed = v
  
def speed_callback(data):
    global speed
    speed = data.data/36  
  
def cbAvoidance(event_msg):
  global avoid
  avoid = event_msg.data




if __name__ == '__main__': 
  try:
    
    rospy.init_node('cmd_vel_to_ackermann_drive')
        
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd_TEB')
    wheelbase = rospy.get_param('~wheelbase', 1.0)
    frame_id = rospy.get_param('~frame_id', 'odom')
    
    rospy.Subscriber('/adaptive_clustering/is_bool', Bool, cbAvoidance, queue_size=1)
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    rospy.Subscriber("/ERP42_speed",Float32,speed_callback)
    pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)
    
    rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/cmd_vel", ackermann_cmd_topic, frame_id, wheelbase)
    
    msg = AckermannDriveStamped()
    speed = 0
    last_time = 0
    avoid = False
    
    kp = 0.3
    ki = 0.1
    kd = 0.0
    mn = 0.0
    mx = 5.55556

    PID_controller = PID(kp, ki, kd, mn, mx)
    PID_controller.reset()
    r = rospy.Rate(30) # 10hz

    v = 0

    while not rospy.is_shutdown():

      vel_error = v - speed
      current_time = rospy.get_time()
      sample_time = current_time - last_time
      last_time = current_time

      vel = PID_controller.step(vel_error, sample_time)
      print("error :",vel_error)
      print("vel :",vel)
      print("speed :", v)
      msg.drive.speed = vel
      pub.publish(msg)
      r.sleep()
        
  except rospy.ROSInterruptException:
    pass

