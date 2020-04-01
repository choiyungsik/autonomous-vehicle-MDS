#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This is a stauto core_controller.py
# Copyright (c) 2020, choiyungsik

import rospy, roslaunch
import numpy as np
import subprocess
import os
import sys
from enum import Enum
from std_msgs.msg import UInt8

class CoreController():
    def __init__(self):
        #subscriber
        self.sub_state_control = rospy.Subscriber('state_transition', UInt8, self.cbEvent, queue_size=1)
        #publisher

        self.Machine_State = Enum('Machine_state', 'parking traffic traffic_red traffic_green cruise')
        self.current_state = self.Machine_State.cruise.value
        self.is_triggered = False
        self.is_green_triggered = False

        loop_rate = rospy.Rate(10)
    
    def cbEvent(self,event_msg):
        rospy.loginfo("The input event is %d",event_msg.data)
        self.is_triggered = True

        if (self.is_green_triggered == False):
            self.current_state = event_msg.data
        elif (self.is_green_triggered == True):
            self.current_state = self.Machine_State.cruise.value


    def fnControl(self):
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
            self.is_green_triggered = False # Turn back on the green_light mode

    def main(self):
        while not rospy.is_shutdown():
            if self.is_triggered == True:
                self.fnControl()

        rospy.spin()

if __name__ == "__main__":
    
    rospy.init_node('Core_Controller')
    node = CoreController()
    node.main()
