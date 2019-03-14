#!/usr/bin/env python

# # # # # # # # # # # # # # # # # # # # # # # # # # #
#						    #
#                   right_arm_sim.py                #
#						    #
# # # # # # # # # # # # # # # # # # # # # # # # # # #

'''
FOR USE IN SIMULATION WITH BAXTER ROBOT ONLY

The right arm simulation file receives commands and operates the right arm of the BAXTER robot.
It can be used in conjunction with the left_arm_sim.py and an instructor file.

The right arm will pick up a brick from a predefined locationa and move it to a central pose to be passed to the left arm.

Brick dimensions are: 0.2 x 0.09 x 0.062 metres
'''

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
        self._limb_name = limb
        self._hover_distance = hover_distance
        self._verbose = verbose				# print debug messages?
        self._sequence = sequence			# will run full demo once enabled
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        self._gripper.set_moving_force(90)			# use 90% of gripper force when moving
        self._gripper.set_holding_force(90)			# use 90% of gripper force when holding
        self._iteration = 0					# which brick is being manipulated: begin at zero
        self._start_angles = {  'right_s0': 0.19194192950377786,
                                'right_s1': -0.4300336661388311,
                                'right_w0': 0.0412353893921642,
                                'right_w1': 0.6860390155959699,
                                'right_w2': -0.5985258112031371,
                                'right_e0': -0.037674601862013546,
                                'right_e1': 1.2950341401339145   }
	# Hardcoded pass angles for horizontal brick placement
        self._h_pass_angles = { 'right_s0': 0.002824106358810141,
                                'right_s1': -1.0514117177590556,
                                'right_w0': 1.102326282212558,
                                'right_w1': 1.70459523735878,
                                'right_w2': 0.7332560190418596,
                                'right_e0': 0.4105871371295322,
                                'right_e1': 1.9241090653363315  }
	# Hardcoded pass angles for vertical brick placement
        self._v_pass_angles = { 'right_s0': -0.17455188835565316,
                                'right_s1': -0.7671991859344809,
                                'right_w0': -0.7748709352937055,
                                'right_w1': 1.572464837389778,
                                'right_w2': -0.7227189655500057,
                                'right_e0': 1.739379551281571,
                                'right_e1': 1.735157459717211  }
        self._hover_angles = None
        self._cpose = Pose()					# calibration pose: empty until calibration function run
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
    	# read commands from instructor.py and execute appropriate function
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
    	funcmap[data.data]()	# execute

    def interpret_leftarm(self, data):
    	# read status updates from left_arm_sim.py and execute appropriate function
    	funcmap = {
        'brick_grabbed': self.releasebrick,
        'brick_placed': self.pickbrick
    	}
    	funcmap[data.data]()	# execute

    def calibrate(self):
     	# zero the end effector relative to the brick to be picked up
     	# user should move the end effector to the zero point BEFORE running the calibration function
     	self._cpose = self._limb.endpoint_pose()
     	self._cpose_angles = self._limb.joint_angles()
     	print("Right arm calibrated... frame origin at:\n{}".format(self._cpose))

    def move_to_start(self:
    	# On initialisation, move to a pre-defined start location
        print('Moving the {0} arm to start pose...'.format(self._limb_name))
        self._guarded_move_to_joint_position(self._start_angles)
        self.gripper_open()

    def move_to_cpose(self):
    	# Move to the recorded calibration pose
        print('Moving the to calibration pose...')
        self._guarded_move_to_joint_position(self._cpose_angles)

    def current_joint_angles(self):
    	# Print current joint angles of right arm
        print('Current joint angles: \n\n {}'.format(self._limb.joint_angles()))

    def ik_request(self, pose):
    	# Convert target position into required joint angles
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
	# Move to passed joint angles, with error message in case of failure
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr('No Joint Angles provided for move_to_joint_positions. Staying put.')

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(2.0)

    def hoverbrick(self):
	# Hover the end effector slightly above the brick spawn point
    	print("Moving to neutral position above brick pile")
    	if self._hover_angles:
	# if this function has already been run, a set of joint angles for this pose will be stored at a class level
    		self._guarded_move_to_joint_position(self._hover_angles)
	    	# in this scenario, simply move to the saved joint angles
    		return
	# Upon running this function the first time since calibration, use Inverse Kinetmatics to find and move to the hover pose
    	calibrationpose = self._cpose
     	neutralpose = Pose()
     	neutralpose.position.x = calibrationpose['position'].x
     	neutralpose.position.y = calibrationpose['position'].y
     	neutralpose.position.z = calibrationpose['position'].z + 0.1	# short distance above the brick spawn point
     	neutralpose.orientation.x = 0
     	neutralpose.orientation.y = 1
     	neutralpose.orientation.z = 0
     	neutralpose.orientation.w = 0
     	joint_angles = self.ik_request(neutralpose)		# use Inverse Kinetmatics to find joint angles
     	self._guarded_move_to_joint_position(joint_angles)
     	self._hover_angles = joint_angles			# save joint angles for future function calls

    def pickbrick(self, brick_y = 0.09, brick_z = 0.062):
	# Pick up the brick to be manipulated
    	calibrationpose = self._cpose
    	self._iteration += 1					# Keep track of which brick is being manipulated
    	self._gripper.open()
    	if self._iteration > 8:							# if all bricks have been picked up
    		rospy.logerr('Reading all 8 bricks have been collected...')	# log a console message
    		return								# do not pick up another
    	print('Collecting brick #{0} from pile'.format(self._iteration))
    	brickpose = Pose()
    	# determine brick location based on calibrationpose and brick dictionary
    	brickpose.position.x = calibrationpose['position'].x
    	brickpose.position.y = calibrationpose['position'].y
    	brickpose.position.z = calibrationpose['position'].z - brick_z		# one brick height below the calibration pose
     	brickpose.orientation.x = 0
     	brickpose.orientation.y = 1
     	brickpose.orientation.z = 0
     	brickpose.orientation.w = 0
    	joint_angles = self.ik_request(brickpose)		# IK solver
    	self._guarded_move_to_joint_position(joint_angles)	# Move to pickup position
    	self._gripper.close()
    	self.hoverbrick()					# Return to neutral hover position
    	pub.publish('brick_picked')				# Inform left arm and brick spawner
        if self._sequence:
            self.movetocenter()					# Move to trade if running full demo

    def movetocenter(self):
    	# Move arm to central trade position with brick
    	if self._iteration in [1, 2, 3, 4, 5]:				# For bricks to be placed vertically
            self._guarded_move_to_joint_position(self._v_pass_angles)
        elif self._iteration in [6, 7, 8]:				# For bricks to be placed horizontally
            self._guarded_move_to_joint_position(self._h_pass_angles)
    	else:
		# Failsafe in case of user error when testing: this should not be reached in the demo
    		rospy.logerr('Brick iteration exceeds expected value')
    		return
    	if self._sequence:						# Inform left arm
    		pub.publish('right_at_center')

    def releasebrick(self):
    	# Let go of the brick being traded, and move away from the trade position
    	self.gripper_open()
    	current_pose = self._limb.endpoint_pose()
    	newpose = Pose()
        newpose.position.x = current_pose['position'].x
        newpose.position.y = current_pose['position'].y
        newpose.position.z = current_pose['position'].z + 0.7*self._hover_distance	# move away so brick does not collide
        newpose.orientation.x = current_pose['orientation'].x
        newpose.orientation.y = current_pose['orientation'].y
        newpose.orientation.z = current_pose['orientation'].z
        newpose.orientation.w = current_pose['orientation'].w
    	joint_angles = self.ik_request(newpose)
    	self._guarded_move_to_joint_position(joint_angles)
    	if self._sequence:						# Inform left arm
    		pub.publish('brick_released')
        self.hoverbrick()						# Return to neutral hover pose above brick spawn point

    def move_to_pos(self):
	# Function to manually move to a given position
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
        ik_pose.position.x = float(raw_input("X Coordinate: "))	# Prompt for user input
        ik_pose.position.y = float(raw_input("Y Coordinate: "))
        ik_pose.position.z = float(raw_input("Z Coordinate: "))
        joint_angles = self.ik_request(ik_pose)
        self._guarded_move_to_joint_position(joint_angles)

    def begin_sequence(self):
	# When user wishes to demo full sequence: reset iteration count, enable communication between arms, and start demo
    	print("BEGINNING FULL DEMO")
        pub.publish('starting_demo')
    	self._sequence = True
    	self._iteration = 0
    	self.pickbrick()


rospy.init_node('right_arm', anonymous=False)		# Create node: only one should be run at once therefore anonymous = False
rospy.wait_for_message('/robot/sim/started', Empty)	# Ensure simulation is running

pub = rospy.Publisher('right_status', String, queue_size=100)	# Topic containing status of the right arm

rightarm = RightArmControl(verbose=False)		# Initialise the right arm
rightarm.move_to_start()				# Move to pre-defined start position
rightarm.calibrate()					# Initial calibration at start position

def listen():
	rospy.Subscriber('instructor_right', String, rightarm.interpret_instructor)	# Wait for command from instructor file
	rospy.Subscriber('left_status', String, rightarm.interpret_leftarm)		# Or react to left arm status updates
	rospy.spin()									# Listen continuously

while not rospy.is_shutdown():				# Main loop
	print("Right arm running...")
	listen()
