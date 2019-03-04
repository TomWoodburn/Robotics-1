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

from std_msgs.msg import (
    Header,
    Empty,
)

import baxter_interface

rightarm_dict = {
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

leftarm_dict = {
    1: 'calibrate',
    2: 'demo',
    3: 'move to cpose',
    4: 'open',
    5: 'close',
    6: 'hover place',
    7: 'place brick',
    8: 'move near center',
    9: 'grab brick',
    10: 'manual move',
    11: 'angles'
}

intro_message = "Please select arm to command:\n\
1 - LEFT - for receiving and placing bricks\n\
2 - RIGHT - for picking and passing bricks\n\n"

rightarm_message = "RIGHT ARM SELECTED...\n\
Please enter one of the following commands: \n\
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

leftarm_message = "LEFT ARM SELECTED...\n\
Please enter one of the following commands: \n\
1 - calibrate - calibrates the right arm to its current position\n\
2 - demo - UNIMPLEMENTED (will run full demo)\n\
3 - move to cpose - returns DENIRO arms to calibration position\n\
4 - open - opens gripper\n\
5 - close - closes gripper\n\
6 - hover place - hover above brick pile\n\
7 - place brick - pick up brick from pile\n\
8 - move near center - move to the trade position\n\
9 - grab brick - release brick and retract\n\
10 - manual move - manually adjust brick position\n\
11 - angles - get current joint angles\n\n"

def instruction():
    pub_l = rospy.Publisher('instructor_left', String, queue_size=10)
    pub_r = rospy.Publisher('instructor_right', String, queue_size=10)
    rospy.init_node('instruction_terminal', anonymous=True)
    while not rospy.is_shutdown():
        print(intro_message)
        try:
            arm = int(raw_input("CHOOSE ARM: "))
            if arm == 1:
                print(leftarm_message)
                while not rospy.is_shutdown():
                    try:
                        command = int(raw_input("ENTER COMMAND: "))
                        if command == 0:
                            break
                        elif command in leftarm_dict:
                            pub_l.publish(leftarm_dict[command])
                        else:
                            print("Value does not correlate with a command.")
                    except:
                        print("Please enter a number.")
            elif arm == 2:
                print(rightarm_message)
                while not rospy.is_shutdown():
                    try:
                        command = int(raw_input("ENTER COMMAND: "))
                        if command == 0:
                            break
                        elif command in rightarm_dict:
                            pub_r.publish(rightarm_dict[command])
                        else:
                            print("Value does not correlate with a command.")
                    except:
                        print("Please enter a number.")
            else:
                print("Invalid arm.")
        except:
            print("Please enter a number")


if __name__ == '__main__':
    try:
        instruction()
    except rospy.ROSInterruptException:
        pass
