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
#from std_msgs.msg import UInt8
#from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

class CoreController():
    def __init__(self):
        #subscriber
        self.sub_avoidance = rospy.Subscriber('avoidanceSW', Bool, self.cbAvoidance, queue_size=1)
        self.sub_traffic = rospy.Subscriber('trafficSW', Bool, self.cbTraffic, queue_size=1)
        self.sub_parking = rospy.Subscriber('parkingSW', Bool, self.cbParking, queue_size=1)
        self.sub_safetyzone = rospy.Subscriber('safetyzoneSW', Bool, self.cbSafetyZone, queue_size=1)
        #self.sub_ackermann = rospy.Subscriber('/ackermann_cmd_state', AckermannDriveStamped, self.cbAckermann, queue_size=1)
        
        
        #publisher
        self.pub_state = rospy.Publisher('/state_graph',dict, queue_size= 1)
        

        self.Machine_State = Enum('Machine_State', 'cruise avoid_cruise parking traffic safety_zone')
        self.current_state = self.Machine_State.cruise.value
        self.Ackermann_data = AckermannDriveStamped()
        self.StateGraph = {'cruise': True, 'avoidance_cruise':False, 'traffic': False, 'parking':False, 'safetyzone':False}            # This value need for starting

        loop_rate = rospy.Rate(10)
    
    def cbAvoidance(self,event_msg):
        rospy.loginfo("Avoidance event is %d",event_msg.data)
        
    
    def cbTraffic(self,event_msg):
        rospy.loginfo('Traffic event is %d',event_msg.data)
    
    def cbParking(self,event_msg):
        rospy.loginfo('Parking event is %d',event_msg.data)

    def cbSafetyZone(self,event_msg):
        rospy.loginfo('SafetyZone event is %d',event_msg.data)


    def fnControl(self):
        if self.current_state == self.Machine_State.cruise.value:
            print('Cruise_mode')
            
        elif self.current_state == self.Machine_State.avoidance_cruise.value:
            print('Avoidance_Cruise_mode')
        
        elif self.current_state == self.Machine_State.parking.value:
            print('Parking_mode')

        elif (self.current_state == self.Machine_State.traffic.value:
            print('Traffic_mode')

        elif self.current_state == self.Machine_State.safety_zone.value:
            print('Safety_Zone_mode')
                 
        
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
