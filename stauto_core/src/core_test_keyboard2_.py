#!/usr/bin/env python
# -*- coding: utf-8 -*-

#This is a stauto core_test_keyboard.py
#Copyright (c) 2020, choiyungsik

import sys, select, termios, tty, math
import rospy
from std_msgs.msg import UInt8
from std_msgs.msg import Bool
from std_msgs.msg import ByteMultiArray
from enum import Enum

header_msgs = """
Make a interruption by typing!!
--------------------------------
c : cruise
o : obstacle detected
p : parking_sign detected
t : traffic_light detected
f : safety_zone detected
r : red_light 
g : green_light
a : straightleft
l : left
s : stop

CTRL_C to quit
"""

def GetKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def fnPublish(key):
    global StateArray, Machine_State, TrafficArray
    for i in StateArray.layout.dim[0].size:
        StateArray.data[i] = False
    for j in TrafficArray.layout.dim[0].size:
        TrafficArray.data[j] = 0

    if key == 'p':
        StateArray.data[Machine_State.parking.value] = True
    elif key == 'c':
        StateArray.data[Machine_State.cruise.value] = True
    elif key == 's':
        StateArray.data[Machine_State.stop.value] = True
    elif key == 'o':
        StateArray.data[Machine_State.avoid_cruise.value] = True
    elif key == 'f':
        StateArray.data[Machine_State.safety_zone.value] = True
    elif str == 'r':
        if StateArray.data[Machine_State.traffic.value] == True:
            TrafficArray.data[0] = 1
        else:
            TrafficArray.data[0] = 0
    elif str == 'g':
        if StateArray.data[Machine_State.traffic.value] == True:
            TrafficArray.data[1] = 1
        else:
            TrafficArray.data[1] = 0
    elif str == 'l':
        if StateArray.data[Machine_State.traffic.value] == True:
            TrafficArray.data[2] = 1
        else:
            TrafficArray.data[2] = 0
    elif str == 's':
        if StateArray.data[Machine_State.traffic.value] == True:
            TrafficArray.data[3] = 1
        else:
            TrafficArray.data[3] = 0

if __name__ == "__main__":
    global Machine_State, StateArray, TrafficArray
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('stauto_core_test_keyboard')

    Machine_State = Enum('Machine_state', 'cruise avoid_cruise stop traffic parking safety_zone')
    state_event = Machine_State.cruise.value
    StateArray = ByteMultiArray()
    TrafficArray = ByteMultiArray()
    StateArray.layout.dim[0].size = 6
    TrafficArray.layout.dim[0].size = 4

    pub_state = rospy.Publisher('/state_graph',ByteMultiArray,queue_size=1)
    pub_traffic = rospy.Publisher('/detect/traffic_sign',ByteMultiArray,queue_size=1)

    loop_rate = rospy.Rate(10) # 10hz

    try:
        print(header_msgs)
        while(1):
            key = GetKey()
            fnPublish(key)

            pub_state.publish(StateArray)
            pub_state.publish(TrafficArray)
            
            if key == '\x03': #CTRL + C
                break

    except Exception as e:
        print(e)
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)