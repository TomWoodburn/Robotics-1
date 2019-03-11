#!/usr/bin/env python

# # # # # # # # # # # # # # # # # # # # # # # # # # #
#                   right_arm_sim.py                #
#              SIMULATION VERSION ONLY              #
#						    #
#    to be used with left_arm_sim and instructor    #
#         and run on top of spawn_solo.py           #
#  						    #
#	       Control BAXTER right arm:	    #
#           - Pick brick from pile                  #
#           - Move brick to swap point              #
#  	    - Trade brick with left arm             #
#    					 	    #
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


class RightArmControl(object):
    def __init__(self, limb='right', hover_distance = 0.25, verbose=True, sequence=False):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._sequence = sequence # bool
        self._limb = baxter_interface.Limb(limb)
        self._limb.set_joint_position_speed(0.05)
        self._gripper = baxter_interface.Gripper(limb)
        self._gripper.set_moving_force(90)
        self._gripper.set_holding_force(90)
        self._iteration = 0		# which brick is picked up next
        self._start_angles = {  'right_s0': 0.18131168311321932,
                                'right_s1': -1.0016043603557936,
                                'right_w0': 0.022976266499086684,
                                'right_w1': 1.2151395716227027,
                                'right_w2': -0.6191932160319986,
                                'right_e0': -0.05272975195082763,
                                'right_e1': 1.338372395790664   }
        self._h_pass_angles = { 'right_s0': -0.004429034804837251,
                                'right_s1': -1.149947770568719,
                                'right_w0': 1.1681167986671221,
                                'right_w1': 1.747705329832261,
                                'right_w2': 0.83886015545,
                                'right_e0': 0.4372348413838414,
                                'right_e1': 1.9127625213615964  }
        self._v_pass_angles = { 'right_s0': -0.12053726662685094,
                                'right_s1': -0.8646772979974182,
                                'right_w0': -0.7067659815851988,
                                'right_w1': 1.8657991319074547,
                                'right_w2': -0.7915199212,
                                'right_e0': 1.8567301606995321,
                                'right_e1': 1.4229690844004566  }
        self._hover_angles = None
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
    	# variable_from_data : function_to_exectute
    	'calibrate' : self.calibrate,
    	'demo' : self.begin_sequence,
    	'move to cpose' : self.move_to_cpose,
    	'open': self.gripper_open,
    	'close': self.gripper_close,
    	'hover brick': self.hoverbrick,
    	'pick brick': self.pickbrick,
    	'move to center': self.movetocenter,
    	'release brick': self.releasebrick,
    	'manual move': self.move_to_pos,
    	'angles': self.current_joint_angles
    	}
    	funcmap[data.data]()

    def interpret_leftarm(self, data):
    	# read data and perform functions accordingly.
    	funcmap = {
        'brick_grabbed': self.releasebrick,
        'brick_placed': self.pickbrick
    	}
    	funcmap[data.data]()

    def calibrate(self):
     	# to be run on calibration request
     	# return a value to calibrate the brick pile location
     	self._cpose = self._limb.endpoint_pose()
     	self._cpose_angles = self._limb.joint_angles()
     	print("Right arm calibrated... frame origin at:\n{}".format(self._cpose))

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

    def hoverbrick(self):
    	print("Moving to neutral position above brick pile")
    	if self._hover_angles:
    		self._guarded_move_to_joint_position(self._hover_angles)
    		return
    	calibrationpose = self._cpose
     	# hover at neutral pose above brick pile
     	neutralpose = Pose()
     	# hoverbrick pose is a short distance above the brick
     	neutralpose.position.x = calibrationpose['position'].x
     	neutralpose.position.y = calibrationpose['position'].y
     	neutralpose.position.z = calibrationpose['position'].z + 0.1
     	neutralpose.orientation.x = 0
     	neutralpose.orientation.y = 1
     	neutralpose.orientation.z = 0
     	neutralpose.orientation.w = 0
     	# CHECK the orientation, ensure arm is ponted downwards
     	joint_angles = self.ik_request(neutralpose)
     	self._guarded_move_to_joint_position(joint_angles)
     	self._hover_angles = joint_angles

    def pickbrick(self, calibrationpose=None, brick_y = 0.09, brick_z = 0.062):
    	if calibrationpose is None:
    		calibrationpose = self._cpose
    	# update number of next brick to take
    	self._iteration += 1
    	# collect the brick - iteration determines position
    	self._gripper.open()
    	rospy.sleep(1.0)
    	# do not continue if al 8 bricks have been moved
    	if self._iteration > 9:
    		rospy.logerr('Reading all 9 bricks have been collected...')
    		return
    	# move above second stack if second group of 4 bricks
    	print('Collecting brick #{0} from pile'.format(self._iteration))
    	brickpose = Pose()
    	# determine brick location based on calibrationpose and brick dictionary
    	brickpose.position.x = calibrationpose['position'].x
    	brickpose.position.y = calibrationpose['position'].y
    	brickpose.position.z = calibrationpose['position'].z - brick_z
     	brickpose.orientation.x = 0
     	brickpose.orientation.y = 1
     	brickpose.orientation.z = 0
     	brickpose.orientation.w = 0
    	joint_angles = self.ik_request(brickpose)
    	self._guarded_move_to_joint_position(joint_angles)
    	self._gripper.close()
    	rospy.sleep(1.0)
    	# return to hover position
    	self.hoverbrick()
    	# inform left arm that brick has been taken if running full sequence
    	pub.publish('brick_picked')
        if self._sequence:
            self.movetocenter()

    def movetocenter(self):
    	# move brick to central position to be obtained by arm
    	if self._iteration in [1, 2, 3, 6, 7, 9]:
            # bricks to be placed vertically
            self._guarded_move_to_joint_position(self._v_pass_angles)
        elif self._iteration in [4, 5, 8]:
            # bricks to be placed horizontally
            self._guarded_move_to_joint_position(self._h_pass_angles)
    	else:
    		# all iterations done - this shouldnt happen in demo
    		rospy.logerr('Brick iteration exceeds expected value')
    		return
    	if self._sequence:
    		pub.publish('right_at_center')

    def releasebrick(self):
    	# let go of brick once held by left arm
    	self.gripper_open()
    	rospy.sleep(1.0)
    	# withdraw arm away from center
    	current_pose = self._limb.endpoint_pose()
    	newpose = Pose()
        newpose.position.x = current_pose['position'].x
        newpose.position.y = current_pose['position'].y #- self._hover_distance
        newpose.position.z = current_pose['position'].z + 0.5*self._hover_distance
        newpose.orientation.x = current_pose['orientation'].x
        newpose.orientation.y = current_pose['orientation'].y
        newpose.orientation.z = current_pose['orientation'].z
        newpose.orientation.w = current_pose['orientation'].w
    	joint_angles = self.ik_request(newpose)
    	self._guarded_move_to_joint_position(joint_angles)
    	rospy.sleep(1.0)
    	# inform left arm that brick has been released if running full sequence
    	if self._sequence:
    		pub.publish('brick_released')
        self.hoverbrick()

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
    	self.pickbrick()


rospy.init_node('right_arm', anonymous=True)

rospy.wait_for_message('/robot/sim/started', Empty)

pub = rospy.Publisher('right_status', String, queue_size=100)

# initialise right arm
rightarm = RightArmControl(verbose=False)
rightarm.move_to_start()
# use start position as initial calibration
rightarm.calibrate()

def listen():
	rospy.Subscriber('instructor_right', String, rightarm.interpret_instructor)
	rospy.Subscriber('left_status', String, rightarm.interpret_leftarm)
	rospy.spin()

while not rospy.is_shutdown():
	print("Right arm running...")
	listen()


