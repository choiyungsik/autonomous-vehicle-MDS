#!/usr/bin/env python
# -*- coding: utf-8 -*-

#This is a stauto core_test_keyboard.py
#Copyright (c) 2020, choiyungsik

import sys, select, termios, tty, math
import rospy
from std_msgs.msg import UInt8
from enum import Enum

header_msgs = """
Make a interruption by typing!!
--------------------------------
p : parking_sign detected
t : traffic_light detected
r : red_light detected
g : green_light detected

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


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('stauto_core_test_keyboard')

    Machine_State = Enum('Machine_state', 'parking traffic traffic_red traffic_green cruise')
    state_event = Machine_State.cruise.value
    
    pub_transition = rospy.Publisher('state_transition',UInt8,queue_size=1)

    loop_rate = rospy.Rate(10) # 10hz

    try:
        print(header_msgs)
        while(1):
            key = GetKey()
            if key == 'p':
                print('Parking sign is detected')
                state_event = Machine_State.parking.value
            
            elif key == 't':
                print('Traffic light is detected')
                state_event = Machine_State.traffic.value

            elif key == 'r':
                print('The traffic light is red!!')
                state_event = Machine_State.traffic_red.value

            elif key == 'g':
                print('The traffic light is turned into green')
                state_event = Machine_State.traffic_green.value
            
            elif key == '\x03': #CTRL + C
                break
            
            pub_transition.publish(state_event)

    except Exception as e:
        print(e)
    
    finally:
        state_event = Machine_State.cruise.value
        pub_transition.publish(state_event)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)