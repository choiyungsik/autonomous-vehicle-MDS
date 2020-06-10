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
from std_msgs.msg import ByteMultiArray, Int32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point

class CoreController():
    def __init__(self):
        #subscriber
        self.sub_traffic = rospy.Subscriber('/detect/traffic_sign', Int32MultiArray, self.cbTraffic, queue_size=1)
        self.sub_avoidance = rospy.Subscriber('/detect/obstacle', Bool, self.cbAvoidance, queue_size=1)
        self.sub_parking = rospy.Subscriber('/detect/parking_sign', Bool, self.cbParking, queue_size=1)
        self.sub_safetyzone = rospy.Subscriber('/detect/safety_sign', Bool, self.cbSafetyZone, queue_size=1)
        self.sub_stop = rospy.Subscriber('/detect/stop_sign',Bool, self.cbStop,queue_size=1)                       #dynamic obstacle mission

        #publisher
        self.pub_state = rospy.Publisher('/state_graph',Int32MultiArray, queue_size=1)

        self.Machine_State = Enum('Machine_State', 'cruise avoid_cruise stop traffic parking safety_zone')
        self.TrafficSign = Enum('TrafficSign','red green left straightleft')
        #self.StopSign = Enum('StopSign','obstacle_stop traffic_stop parking_stop')

        self.StateGraph = Int32MultiArray()
        self.StateGraph.layout.dim.append(MultiArrayDimension())
        self.StateGraph.layout.dim[0].label = "state_graph"
        self.StateGraph.layout.dim[0].size = 6
        self.StateGraph.layout.dim[0].stride = 6
        self.StateGraph.layout.data_offset = 0
        self.StateGraph.data=[0]*6
        for i in range(6):
            if i == 1:
                self.StateGraph.data[i] = 1
            else:
                self.StateGraph.data[i] = 0

        self.cur_state = self.Machine_State.cruise.value
        self.cur_traffic = self.TrafficSign.green.value
        self.stop_flag = False

        loop_rate = rospy.Rate(10)

    def cbTraffic(self,event_msg):
        self.fnDecideMode(self.Machine_State.traffic.value,event_msg)
        if event_msg.data[self.TrafficSign.red.value-1] == 1:
            self.cur_traffic = self.TrafficSign.red.value -1
        elif event_msg.data[self.TrafficSign.green.value-1] == 1:
            self.cur_traffic = self.TrafficSign.green.value -1
        elif event_msg.data[self.TrafficSign.left.value-1] == 1:
            self.cur_traffic = self.TrafficSign.left.value -1
        elif event_msg.data[self.TrafficSign.straightleft.value-1] == 1:
            self.cur_traffic = self.TrafficSign.straightleft.value -1

    def cbStop(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.stop.value,0)

    def cbAvoidance(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.avoid_cruise.value,0)

    def cbParking(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.parking.value,0)

    def cbSafetyZone(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.safety_zone.value,0)
    
    def fnDecideMode(self,mode,sub_event):
        for i in range(6):
            self.StateGraph.data[i] = 0
        print('cruise_state : ', self.Machine_State.cruise.value)

        if mode == self.Machine_State.cruise.value:
            self.cur_state = self.Machine_State.cruise.value
            print('Cruise')
            
        elif (mode == self.Machine_State.stop.value) and (self.cur_traffic == 0):
            self.cur_state = self.Machine_State.stop.value
            print('Stop')

        elif mode == self.Machine_State.traffic.value:
            if (sub_event.data[self.TrafficSign.red.value -1] == 1) and (self.cur_traffic != 0):
                self.cur_state = self.Machine_State.traffic.value
                self.cur_traffic = 0
                print('Traffic Red')
            elif sub_event.data[self.TrafficSign.green.value -1] == 1:
                self.cur_state = self.Machine_State.cruise.value 
                self.cur_traffic = 1
                print('Traffic Green')
            elif sub_event.data[self.TrafficSign.left.value -1] == 1:
                self.cur_state = self.Machine_State.cruise.value 
                self.cur_traffic = 2
                print('Traffic Left')
            elif sub_event.data[self.TrafficSign.straightleft.value -1] == 1:
                self.cur_state = self.Machine_State.cruise.value
                self.cur_traffic = 3
                print('Traffic StraightLeft')

        elif mode == self.Machine_State.avoid_cruise.value:
            self.cur_state = self.Machine_State.avoid_cruise.value
            print('Avoidance_Cruise')
        elif mode == self.Machine_State.parking.value:
            self.cur_state = self.Machine_State.parking.value
            print('parking')
        elif mode == self.Machine_State.safety_zone.value:
            self.cur_state = self.Machine_State.safety_zone.value
            print('Safety_Zone')
        
        self.StateGraph.data[self.cur_state - 1] = 1
        self.fnPublishMode()

    def fnPublishMode(self):
        if self.StateGraph.data[self.Machine_State.cruise.value-1] == 1:
            print('Cruise_mode')
        elif self.StateGraph.data[self.Machine_State.avoid_cruise.value-1] == 1:
            print('Avoidance_Cruise_mode')
        elif self.StateGraph.data[self.Machine_State.stop.value-1] == 1:
            print('Stop_mode')
        elif self.StateGraph.data[self.Machine_State.traffic.value-1] == 1:
            print('Traffic_mode')
        elif self.StateGraph.data[self.Machine_State.parking.value-1] == 1:
            print('Parking_mode')
        elif self.StateGraph.data[self.Machine_State.safety_zone.value-1] == 1:
            print('Safety_Zone_mode')
        
        self.pub_state.publish(self.StateGraph)
        
    def main(self):
        rospy.spin()
        # while not rospy.is_shutdown():
        #     self.fnPublishMode()

if __name__ == "__main__":
    
    rospy.init_node('Core_Controller')
    node = CoreController()
    node.main()

