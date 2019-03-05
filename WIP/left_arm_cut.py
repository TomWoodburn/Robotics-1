#!/usr/bin/env python

# modified version of left arm, cut down

# # # # # # # # # # # # # # # # # # # # # # # # # # #
#                    left_arm.py                    #
#                                                   #
#     to be used with right_arm and instructor      #
#                                                   #
#  			Control DENIRO left arm:				#
#            - Move above structure area            #
#  			 - Erect structure                      #
#    					 							#
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
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface

class LeftArmControl(object):
    def __init__(self, limb='left', hover_distance = 0.1, verbose=True, sequence=False):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._sequence = sequence # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        self._gripper.set_moving_force(100)
        self._gripper.set_holding_force(100)
        self._iteration = 1		# which brick is picked up next
        self._start_angles = {	'left_w0': 0.5816,
        					  	'left_w1': 1.3703,
        					  	'left_w2': 1.7250,
        					  	'left_e0': -0.7168,
                         	  	'left_e1': 0.9926,
                         	  	'left_s0': -0.0343,
                         	  	'left_s1': -0.6207,   }
        self._close_angles = {  'left_w0': -1.1729,
                                'left_w1': 1.9277,
                                'left_w2': 2.2930,
                                'left_e0': -0.2981,
                                'left_e1': 1.9899,
                                'left_s0': 0.0264,
                                'left_s1': -1.3085 }
        self._h_grab_angles = { 'left_w0': -1.24279,
                                'left_w1': 1.52200,
                                'left_w2': 2.39106,
                                'left_e0': -0.31283,
                                'left_e1': 1.52945,
                                'left_s0': -0.37826,
                                'left_s1': -0.65179 }
        self._v_grab_angles = { 'left_w0': 0.79768,
                                'left_w1': 1.34974,
                                'left_w2': 1.30512,
                                'left_e0': -2.03880,
                                'left_e1': 1.38635,
                                'left_s0': 0.11396,
                                'left_s1': 0.63172 }
        self._hover_pose = None
        # create empty pose and angles index for calibration
        self._cpose = Pose()
        self._cpose_angles = {}
        ns = 'ExternalTools/' + limb + '/PositionKinematicsNode/IKService'
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print('Getting robot state... ')
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print('Enabling robot...')
        self._rs.enable()

    def interpret_instructor(self, data):
    	# read data and perform functions accordingly.
    	funcmap = {
    #   variable_from_data : function_to_exectute
    	'calibrate' : self.calibrate,
    	'demo' : self.begin_sequence,
    	'move to cpose' : self.move_to_cpose,
    	'open': self.gripper_open,
    	'close': self.gripper_close,
    	'hover place': self.hoverplace,
    	'place brick': self.placebrick,
    	'move near center': self.movenearcenter,
    	'grab brick': self.takebrick,
    	'manual move': self.move_to_pos,
    	'angles': self.current_joint_angles
    	}
    	funcmap[data.data]()

    def interpret_rightarm(self, data):
        funcmap = {
        }
        try:
            funcmap[data.data]()
        except:
            rospy.logerr('Function {} not in dictionary'.format(data.data))

    def calibrate(self):
     	# to be run on calibration request
     	# return a value to calibrate the brick pile location
     	self._cpose = self._limb.endpoint_pose()
     	self._cpose_angles = self._limb.joint_angles()
     	print("Left arm calibrated... frame origin at:\n{}".format(self._cpose))
        self._hover_pose = None
     	return

    def move_to_start(self, start_angles=None):
    	# On initialisation, move to start pos for calibration
    	if start_angles is None:
    		start_angles = self._start_angles
        print('Moving the {0} arm to start pose...'.format(self._limb_name))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)

    def move_to_cpose(self):
    	# Move to start pos for calibration
        print('Moving the to calibration pose...')
        self._guarded_move_to_joint_position(self._cpose_angles)

    def current_joint_angles(self):
    	# get current joint angles
        print('Current joint angles: \n\n {}'.format(self._limb.joint_angles()))

    def ik_request(self, pose):
    	# convert target position into required joint angles
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr('Service call failed: %s' % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print('IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}'.format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print('IK Joint Solution:\n{0}'.format(limb_joints))
                print('------------------')
        else:
            rospy.logerr('INVALID POSE - No Valid Joint Solution Found.')
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr('No Joint Angles provided for move_to_joint_positions. Staying put.')

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def hoverplace(self):
    	print("Moving to neutral position by structure")
     	# hover at neutral pose above brick pile
        if self._hover_pose:
            joint_angles = self.ik_request(self._hover_pose)
            self._guarded_move_to_joint_position(joint_angles)
            return
        calibrationpose = self._cpose
     	neutralpose = Pose()
     	# hoverbrick pose is a short distance above the brick
     	neutralpose.position.x = calibrationpose['position'].x + self._hover_distance
     	neutralpose.position.y = calibrationpose['position'].y
     	neutralpose.position.z = calibrationpose['position'].z + self._hover_distance
        neutralpose.orientation.x = 1
        neutralpose.orientation.y = 1
        neutralpose.orientation.z = 0
        neutralpose.orientation.w = 0
     	# CHECK the orientation, ensure arm is ponted downwards
     	joint_angles = self.ik_request(neutralpose)
     	self._guarded_move_to_joint_position(joint_angles)
        self._hover_pose = neutralpose

    def placebrick(self, calibrationpose=None, bx = 0.2, by = 0.09, bz=0.062):
    	if calibrationpose is None:
    		calibrationpose = self._cpose
    	# collect the brick - iteration determines position
    	brickdict = {
   #brick : [xpos, ypos, zpos, xor, yor, zor, wor]
    	1 : [0, -0.9*bx , bx       ],               
    	2 : [0, 0       , bx       ],     
    	3 : [0, 0.9*bx  , bx       ],          
    	4 : [0, -0.52*bx, bx+bz    ],          
    	5 : [0, 0.52*bx , bx+bz    ],  
    	6 : [0, -0.52*bx, 2*bx+bz  ],  
    	7 : [0, 0.52*bx , 2*bx+bz  ],
    	8 : [0, 0       , 2*bx+2*bz]
    	}
    	# do not continue if al 8 bricks have been moved
    	if self._iteration > 8:
    		rospy.logerr('Reading all 8 bricks have been placed...')
    		return
        self.hoverplace()
    	brickpose = Pose()
    	# determine brick location based on calibrationpose and brick dictionary
    	brickpose.position.x = calibrationpose['position'].x
    	brickpose.position.y = calibrationpose['position'].y + brickdict[self._iteration][1]
    	brickpose.position.z = calibrationpose['position'].z + brickdict[self._iteration][2] + 0.1
     	brickpose.orientation.x = 1
        brickpose.orientation.y = 1
        brickpose.orientation.z = 0
        brickpose.orientation.w = 0
    	joint_angles = self.ik_request(brickpose)
    	self._guarded_move_to_joint_position(joint_angles)
        brickpose.position.z = calibrationpose['position'].z + brickdict[self._iteration][2]
        joint_angles = self.ik_request(brickpose)
        self._guarded_move_to_joint_position(joint_angles)
    	self._gripper.open()
    	rospy.sleep(1.0)
        brickpose.position.z = calibrationpose['position'].z + brickdict[self._iteration][2] + 0.1
        joint_angles = self.ik_request(brickpose)
        self._guarded_move_to_joint_position(joint_angles)
    	# return to calibration position
    	self.hoverplace()
    	# inform left arm that brick has been taken if running full sequence
    	if self._sequence:
    		pub.publish('brick placed')

    def movenearcenter(self):
    	# move brick to central position to be obtained by arm
        centerpose = Pose()
        centerpose.position.x = 0.5
        centerpose.position.y = 0.2
        centerpose.position.z = 0.5
        centerpose.orientation.x = 1
        centerpose.orientation.y = 1
        centerpose.orientation.z = -1
        centerpose.orientation.w = 1
        joint_angles = self.ik_request(centerpose)
        self._guarded_move_to_joint_position(joint_angles)
        #self._guarded_move_to_joint_position(self._close_angles)
        self.gripper_open()

    def takebrick(self):
        # update number of next brick to place
        self._iteration += 1
    	# withdraw arm away from center
        if self._iteration in [1, 2, 3, 6, 7, 9]:
            brickpose = Pose()
            brickpose.position.x = 0.5
            brickpose.position.y = 0
            brickpose.position.z = 0.5
            brickpose.orientation.x = 1
            brickpose.orientation.y = 1
            brickpose.orientation.z = -1
            brickpose.orientation.w = 1
            joint_angles = self.ik_request(brickpose)
        elif self._iteration in [4, 5, 8]:
            brickpose = Pose()
            brickpose.position.x = 0.5
            brickpose.position.y = -0.02
            brickpose.position.z = 0.5
            brickpose.orientation.x = 1
            brickpose.orientation.y = 1
            brickpose.orientation.z = -1
            brickpose.orientation.w = 1
            joint_angles = self.ik_request(brickpose)
        else:
            # all iterations done - this shouldnt happen in demo
            ros.logerr('Brick iteration exceeds expected value')
            return
        self._guarded_move_to_joint_position(joint_angles)
        self.gripper_close()
        rospy.sleep(1.0)
    	# inform left arm that brick has been released if running full sequence
    	if self._sequence:
    		pub.publish('left grabbed')

    def move_to_pos(self):
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        print("Current pose: \n {}".format(ik_pose))
        # choose new position
        ik_pose.position.x = float(raw_input("X Coordinate: "))
        ik_pose.position.y = float(raw_input("Y Coordinate: "))
        ik_pose.position.z = float(raw_input("Z Coordinate: "))
        joint_angles = self.ik_request(ik_pose)
        self._guarded_move_to_joint_position(joint_angles)

    def begin_sequence(self):
    	print("BEGINNING FULL DEMO")
    	self._sequence = True
    	self._iteration = 0


rospy.init_node('left_arm', anonymous=True)

rospy.wait_for_message('/robot/sim/started', Empty)

pub = rospy.Publisher('left_status', String, queue_size=5)

# initialise left arm
leftarm = LeftArmControl(verbose=False)
leftarm.move_to_start()
# use start position as initial calibration
leftarm.calibrate()

def listen():
	rospy.Subscriber('instructor_left', String, leftarm.interpret_instructor)
	rospy.Subscriber('right_status', String, leftarm.interpret_rightarm)
	rospy.spin()

while not rospy.is_shutdown():
	print("Left arm running...")
	listen()

