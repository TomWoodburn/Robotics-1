#!/usr/bin/env python

# # # # # # # # # # # # # # # # # # # # # # # # # # #
#                  instructor.py                    #
#                                                   #
#       Test functionality of individual arms       #
#                  and run full demo                #
#                                                   #
# # # # # # # # # # # # # # # # # # # # # # # # # # #

# brick dims: 0.2 x 0.09 x 0.062 metres

import argparse
import struct
import sys
import copy

import rospy
import rospkg

from std_msgs.msg import String

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

from std_msgs.msg import (
    Header,
    Empty,
)

import baxter_interface

function_dict = {
    1: 'calibrate',
    2: 'demo',
    3: 'move to cpose',
    4: 'open',
    5: 'close',
    6: 'hover brick',
    7: 'pick brick',
    8: 'move to center',
    9: 'release brick',
    10: 'manual move',
    11: 'angles'
}

instruction_message = "Please enter one of the following commands: \n\
1 - calibrate - calibrates the right arm to its current position\n\
2 - demo - UNIMPLEMENTED (will run full demo)\n\
3 - move to cpose - returns DENIRO arms to calibration position\n\
4 - open - opens gripper\n\
5 - close - closes gripper\n\
6 - hover brick - hover above brick pile\n\
7 - pick brick - pick up brick from pile\n\
8 - move to center - move to the trade position\n\
9 - release brick - release brick and retract\n\
10 - manual move - manually adjust brick position\n\
11 - angles - get current joint angles\n\n"

print(instruction_message)

def instruction():
    pub = rospy.Publisher('instructor', String, queue_size=10)
    rospy.init_node('instruction_terminal', anonymous=True)
    while not rospy.is_shutdown():
        command_raw = raw_input("ENTER COMMAND: ")
        if len(command_raw) < 3:
            command_num = int(command_raw)
            if command_num in function_dict:
                command = function_dict[command_num]
            else:
                print("Number does not correlate with a command")
                continue
        elif command_raw in function_dict.values():
            command = command_raw
        else:
            print("Invalid function")
            continue
        # print("ISSUING COMMAND: {}".format(command))
        pub.publish(command)

if __name__ == '__main__':
    try:
        instruction()
    except rospy.ROSInterruptException:
        pass
