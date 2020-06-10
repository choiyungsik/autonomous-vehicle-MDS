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
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import ByteMultiArray
from geometry_msgs.msg import Point

class CoreController():
    def __init__(self):
        #subscriber
        self.sub_state = rospy.Subscriber('/state_graph', ByteMultiArray, self.cbState, queue_size=1)
        self.sub_traffic = rospy.Subscriber('/detect/traffic_sign', ByteMultiArray, self.cbTraffic, queue_size=1)

        #publisher
        self.pub_state = rospy.Publisher('/state_graph',ByteMultiArray, queue_size=1)

        self.Machine_State = Enum('Machine_State', 'cruise avoid_cruise stop traffic parking safety_zone')
        self.TrafficSign = Enum('TrafficSign','red green left straightleft')

        self.StateGraph = [True,False,False,False,False,False]              # This value need for starting
        self.TrafficGraph = [False,False,False,False]
        self.cur_state = self.Machine_State.cruise.value

        loop_rate = rospy.Rate(10)
    
    def cbState(self,mode):
        #rospy.loginfo('State event',mode)
        self.fnDecideMode(mode,0)

    def cbTraffic(self,mode):
        #rospy.loginfo('Traffic event is ',mode)
        self.fnDecideMode(0,mode)
    
    def fnDecideMode(self,mode1,mode2):
        for i in self.Machine_State.safety_zone.value:
            self.StateGraph[i] = False
        for j in self.TrafficSign.straightleft.value:
            self.TrafficGraph[j] = False
        self.cur_state = mode

        if mode1.data[Machine_State.cruise.value] == True:
            print('Cruise')
        elif mode1.data[Machine_State.stop.value] == True:
            print('Stop')
        elif mode1.data[Machine_State.traffic.value] == True:
            if mode2.data[TrafficSign.red.value] == 1:
                print('Traffic Red')
            elif mode2.data[TrafficSign.green.value] == 1:
                print('Traffic Green')
                mode = self.Machine_State.cruise.value
            elif mode2.data[TrafficSign.left.value] == 1:
                print('Traffic Left')
                mode = self.Machine_State.cruise.value
            elif mode2.data[TrafficSign.straightleft.value] == 1:
                print('Traffic StraightLeft')
                mode = self.Machine_State.cruise.value
        elif mode1.data[Machine_State.avoid_cruise.value] == True:
            print('Avoidance_Cruise')
        elif mode1.data[Machine_State.parking.value] == True:
            print('parking')
        elif mode1.data[Machine_State.safety_zone.value] == True:
            print('Safety_Zone')

    def fnPublishMode(self):
        if self.StateGraph[self.Machine_State.cruise.value] == True:
            print('Cruise_mode')
        elif self.StateGraph[self.Machine_State.avoid_cruise.value] == True:
            print('Avoidance_Cruise_mode')
        elif self.StateGraph[self.Machine_State.stop.value] == True:
            print('Parking_mode')
        elif self.StateGraph[self.Machine_State.traffic.value] == True:
            print('Traffic_mode')
        elif self.StateGraph[self.Machine_State.parking.value] == True:
            print('Safety_Zone_mode')
        elif self.StateGraph[self.Machine_State.safety_zone.value] == True:
            print('Safety_Zone_mode')
        
        self.pub_state.publish(self.StateGraph)
        
    def main(self):
        rospy.spin()

if __name__ == "__main__":
    
    rospy.init_node('Core_Controller')
    node = CoreController()
    node.main()
