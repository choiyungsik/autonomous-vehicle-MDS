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
from std_msgs.msg import UInt8, Int32
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import ByteMultiArray, Int32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point , PoseStamped

class CoreController():
    def __init__(self):
        #subscriber
        self.sub_traffic = rospy.Subscriber('/detect/traffic_sign', Int32MultiArray, self.cbTraffic, queue_size=1)
        self.sub_avoidance = rospy.Subscriber('/adaptive_clustering/is_bool', Bool, self.cbAvoidance, queue_size=1)
        self.sub_parking = rospy.Subscriber('/detect/parking_sign', Bool, self.cbParking, queue_size=1)
        self.sub_safetyzone = rospy.Subscriber('/detect/safety_sign', Bool, self.cbSafetyZone, queue_size=1)
        self.sub_crosswalk = rospy.Subscriber('/detect/crosswalk_sign', Bool, self.cdCrosswalk, queue_size=1)
        self.sub_stop = rospy.Subscriber('stop_line',Int32, self.cbStop,queue_size=1)                       #dynamic obstacle mission
        self.sub_cruise = rospy.Subscriber('/detect/cruise',Bool, self.cbcruise, queue_size=1)
        self.sub_backup = rospy.Subscriber('current_step', PoseStamped, self.cbBackup, queue_size=1)

        #publisher
        self.pub_state = rospy.Publisher('/state_machine',Int32MultiArray, queue_size=1)

        #self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

        self.Machine_State = Enum('Machine_State', 'cruise avoid_cruise stop traffic parking safety_zone crosswalk backup')
        self.TrafficSign = Enum('TrafficSign','red green left straightleft')
        #self.StopSign = Enum('StopSign','obstacle_stop traffic_stop parking_stop crosswalk_stop')

        self.StateGraph = Int32MultiArray()
        self.StateGraph.layout.dim.append(MultiArrayDimension())
        self.StateGraph.layout.dim[0].label = "state_graph"
        self.StateGraph.layout.dim[0].size = 7
        self.StateGraph.layout.dim[0].stride = 7
        self.StateGraph.layout.data_offset = 0
        self.StateGraph.data=[0]*7
        for i in range(7):
            if i == 1:
                self.StateGraph.data[i] = 1
            else:
                self.StateGraph.data[i] = 0

        self.cur_state = self.Machine_State.cruise.value
        self.backup_state = self.Machine_State.cruise.value
        self.cur_traffic = self.TrafficSign.green.value
        self.stop_flag = False
        self.stop_line = 0

        self.avoid_count = 0
        self.avoid_flag = 0

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
        print('stop_line : ', event_msg.data)
        self.stop_line = event_msg.data

    def cbAvoidance(self,event_msg):
        if event_msg.data == True:
            self.avoid_flag = 1
            self.fnDecideMode(self.Machine_State.avoid_cruise.value,self.avoid_count)
            #self.avoid_time = rospy.get_rostime()
        if event_msg.data == False:
            if self.avoid_flag == 1:
                self.avoid_count += 1
                #print("ooooooooooooooo",self.avoid_count)
                if self.avoid_count >= 3:
                    self.avoid_count = 0
                    self.avoid_flag = 0
                    self.fnDecideMode(self.Machine_State.cruise.value,0)
            elif self.avoid_flag == 0:
                self.fnDecideMode(self.Machine_State.cruise.value,0)
            #if(rospy.get_rostime() - self.avoid_time >= 1.5):
            #    self.fnDecideMode(self.Machine_State.cruise.value,0)

    def cbParking(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.parking.value,0)

    def cbcruise(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.cruise.value,0)

    def cbSafetyZone(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.safety_zone.value,0)

    def cdCrosswalk(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.crosswalk.value,0)
    
    def cbBackup(self,event_msg):
        step_num = event_msg.pose.position.z
        print('step num : ' ,step_num)
        #event_msg.data[self.TrafficSign.red.value-1] == 1
        if(15 <= step_num and step_num < 40 ):
            self.backup_state = self.Machine_State.traffic.value
            #print('Traffic section')
        
        #elif(50 <= step_num and step_num < 70):
        #    self.backup_state = self.Machine_State.avoid_cruise.value
        #    print('Avoidance section')
        #elif(71 <= step_num and step_num < 79):
        #    self.backup_state = self.Machine_State.safety_zone.value
        #    print('Safety Zone section')
        #elif(80 <= step_num and step_num < 120):
        #    self.backup_state = self.Machine_State.safety_zone.value
        #    print('Safety Zone section')
        #elif(80 <= step_num and step_num < 120):
        #    self.backup_state = self.Machine_State.crosswalk.value
        #    print('Cross Walk section')
        #elif(80 <= step_num and step_num < 120):
        #    self.backup_state = self.Machine_State.traffic.value
        #    print('Traffic section')
        
        # else:
        #     self.backup_state = self.Machine_State.cruise.value

        # self.fnDecideMode(self.Machine_State.backup.value,self.backup_state)

        if(15 <= step_num and step_num < 27 ):
            self.backup_state = self.Machine_State.traffic.value
            print('Traffic section')
        elif(30 <= step_num and step_num < 35):
            self.backup_state = self.Machine_State.avoid_cruise.value
            print('Avoidance section')
        elif(40 <= step_num and step_num < 45):
            self.backup_state = self.Machine_State.safety_zone.value
            print('Safety Zone section')
        elif(80 <= step_num and step_num < 120):
            self.backup_state = self.Machine_State.safety_zone.value
            print('Safety Zone section')
        elif(80 <= step_num and step_num < 120):
            self.backup_state = self.Machine_State.crosswalk.value
            print('Cross Walk section')
        elif(80 <= step_num and step_num < 120):
            self.backup_state = self.Machine_State.traffic.value
            print('Traffic section')
        else:
            self.backup_state = self.Machine_State.cruise.value

        # self.fnDecideMode(self.Machine_State.backup.value,self.backup_state)
    
    # def timer_callback(self):
    #     self.cur_state = self.backup_state


    def fnDecideMode(self,mode,sub_event):
        for i in range(7):
            self.StateGraph.data[i] = 0
        #print('cruise_state : ', self.Machine_State.cruise.value)

        if mode == self.Machine_State.cruise.value:
            self.cur_state = self.Machine_State.cruise.value
            #print('Cruise')

        elif mode == self.Machine_State.traffic.value:
            self.cur_state = self.Machine_State.traffic.value
            print('Cruise')
            
        if (mode == self.Machine_State.stop.value) and (self.cur_state == self.Machine_State.avoid_cruise.value):
            self.cur_state = self.Machine_State.stop.value
            print('Stop')

        elif (mode == self.Machine_State.stop.value) and (self.cur_state == self.Machine_State.parking.value):
            self.cur_state = self.Machine_State.stop.value
            print('Stop')

        elif (mode == self.Machine_State.stop.value) and (self.cur_state == self.Machine_State.safety_zone.value):
            self.cur_state = self.Machine_State.stop.value
            print('Stop')

        elif (mode == self.Machine_State.stop.value) and (self.cur_state == self.Machine_State.crosswalk.value):
            self.cur_state = self.Machine_State.stop.value
            print('Stop')

        elif mode == self.Machine_State.traffic.value:
            if (sub_event.data[self.TrafficSign.red.value -1] == 1) and (self.cur_traffic != 0):
                self.cur_state = self.Machine_State.traffic.value
                self.cur_traffic = 0
                print('Traffic Red')
            elif (sub_event.data[self.TrafficSign.red.value -1] == 1) and (self.cur_traffic ==0):
                if self.stop_line == 1:
                    self.cur_state = self.Machine_State.stop.value
                    print('stop')
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
            self.avoid_count = self.avoid_count + 1 
            #print('Avoidance_Cruise')

        elif mode == self.Machine_State.parking.value:
            self.cur_state = self.Machine_State.parking.value
            #print('Parking')

        elif mode == self.Machine_State.safety_zone.value:
            self.cur_state = self.Machine_State.safety_zone.value
            #print('Safety_Zone')

        elif mode == self.Machine_State.crosswalk.value:
            self.cur_state = self.Machine_State.crosswalk.value
            #print('Crosswalk')

 
        # if(self.cur_state != self.backup_state):
        #     self.cur_state = self.backup_state
        
        #if (self.stop_line == 1) and (self.cur_state == self.Machine_State.avoid_cruise.value):
        #    self.cur_state = self.Machine_State.stop.value
        #    print('Stop')

        if (self.stop_line == 1) and (self.cur_state == self.Machine_State.parking.value):
            self.cur_state = self.Machine_State.stop.value
            #print('Stop')

        #elif (self.stop_line == 1) and (self.cur_state == self.Machine_State.safety_zone.value):
        #    self.cur_state = self.Machine_State.stop.value
        #    print('Stop')

        elif (self.stop_line == 1) and (self.cur_state == self.Machine_State.crosswalk.value):
            self.cur_state = self.Machine_State.stop.value
            #print('Stop')
        #print(sub_event, sub_event.data, "ddddddd")
        #print(sub_event.msg)
        if self.cur_state == self.Machine_State.traffic.value:
            if (self.cur_traffic == 0) and (self.stop_line == 1):
                self.stop_flag=True
            
            if self.stop_flag==True:
                if self.cur_traffic == 0:
                    self.cur_state = self.Machine_State.stop.value
                    print('stop')
                else:
                    self.cur_state = self.Machine_State.traffic.value
                    self.stop_flag=False
            if self.stop_line == 1:
                self.cur_state =self.Machine_State.stop.value
            print('Crosswalk')
            
        # elif mode == self.Machine_State.backup.value:
        #     self.cur_state = sub_event
            
 
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
        elif self.StateGraph.data[self.Machine_State.crosswalk.value-1] == 1:
            print('Crosswalk_mode')
        
        self.pub_state.publish(self.StateGraph)
        
    def main(self):
        rospy.spin()
        # while not rospy.is_shutdown():
        #     self.fnPublishMode()

if __name__ == "__main__":
    
    rospy.init_node('Core_Controller')
    node = CoreController()
    node.main()
