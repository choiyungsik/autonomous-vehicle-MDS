#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This is a stauto core_controller.py
# Copyright (c) 2020, choiyungsik

import rospy, roslaunch
import numpy as np
import subprocess
import os
import sys
import rospkg
import math
from enum import Enum
from std_msgs.msg import UInt8
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point

class CoreController():
    def __init__(self):
        #subscriber
        self.sub_state_control = rospy.Subscriber('state_transition', UInt8, self.cbEvent, queue_size=1)
        self.sub_ackermann = rospy.Subscriber('/ackermann_cmd_state', AckermannDriveStamped, self.cbAckermann, queue_size=1)
        
        #publisher
        self.pub_final_ackermann = rospy.Publisher('/ackermann_cmd',AckermannDriveStamped, queue_size= 1)

        self.Machine_State = Enum('Machine_state', 'parking traffic traffic_red traffic_green cruise')
        self.current_state = self.Machine_State.cruise.value
        self.Ackermann_data = AckermannDriveStamped()
        self.is_triggered = False                       # This value need for starting
        self.is_green_triggered = False

        loop_rate = rospy.Rate(10)
    
    def cbEvent(self,event_msg):
        rospy.loginfo("The input event is %d",event_msg.data)
        self.is_triggered = True

        if (self.is_green_triggered == False):
            self.current_state = event_msg.data
        elif (self.is_green_triggered == True):
            self.current_state = self.Machine_State.cruise.value

    def cbAckermann(self,event_msg):
        #rospy.loginfo("The ackermann_steer_angel is %f ",event_msg.drive.steering_angle)

        if self.current_state == self.Machine_State.cruise.value:
            self.Ackermann_data.drive.speed = event_msg.drive.speed
            self.Ackermann_data.drive.steering_angle = event_msg.drive.steering_angle
        #self.Ackermann_data = event_msg
        elif (self.current_state == self.Machine_State.parking.value) or (self.current_state == self.Machine_State.traffic.value) or (self.current_state == self.Machine_State.traffic_red.value) :
            self.Ackermann_data.drive.speed = 0
            self.Ackermann_data.drive.steering_angle = 0
        #print("Ackermann_data : ", self.Ackermann_data)


    def fnControl(self):
        print("Ackerman_data : " , self.Ackermann_data)
        if self.current_state == self.Machine_State.parking.value:
            print('In core, Parking Mode Running!!')
            

        elif self.current_state == self.Machine_State.traffic.value:
            print('In core, Traffic Mode Running!!')
        
        elif self.current_state == self.Machine_State.traffic_red.value:
            print('The core,In Traffic Mode RED light is on')

        elif (self.current_state == self.Machine_State.traffic_green.value) and (self.is_green_triggered == False):
            print('GREEN light is on!! Turning back to CRUISE MODE')

            self.is_green_triggered = True
            self.current_state = self.Machine_State.cruise.value
            #rospy.sleep(1)

        elif self.current_state == self.Machine_State.cruise.value:
            print('In Core, Cruise Mode!!')
            self.is_green_triggered = False     # Turn back on the green_light mode
        
        self.pub_final_ackermann.publish(self.Ackermann_data)
        
    def main(self):
        while not rospy.is_shutdown():
            if self.is_triggered == True:
                self.fnControl()

        #rospy.spin()

if __name__ == "__main__":
    
    rospy.init_node('Core_Controller')
    node = CoreController()
    node.main()
