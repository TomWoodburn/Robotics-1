#!/usr/bin/env python

# # # # # # # # # # # # # # # # # # # # # # # # # # #
#						    #
#                    left_arm.py                    #
#                                                   #
# # # # # # # # # # # # # # # # # # # # # # # # # # #

'''
FOR USE WITH THE PHYSICAL DENIRO ROBOT

The left arm file receives commands and operates the left arm of DENIRO.
It should be used in conjunction with the right_arm.py and an instructor file.

The left arm will navigate the structure area, receive bricks from the right arm, and place them in predefined locations.

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

class LeftArmControl(object):
    def __init__(self, limb='left', hover_distance = 0.1, verbose=True, sequence=False):
        self._limb_name = limb
        self._hover_distance = hover_distance
        self._verbose = verbose					# print debug messages?
        self._sequence = sequence				# will run full demo once enabled
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        self._gripper.set_moving_force(100)			# use 100% of gripper force when moving
        self._gripper.set_holding_force(100)			# use 100% of gripper force when holding
        self._iteration = 0					# which brick has been reached: begin at zero
        self._start_angles = {  'left_w0': 0.3265927118184573,
                                'left_w1': 1.0432241011396997,
                                'left_w2': 1.7990492228330535,
                                'left_e0': -0.5803752203184978,
                                'left_e1': 1.654344222960436,
                                'left_s0': 0.11933210656763382,
                                'left_s1': -1.0294956526537833  }
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
    	# read commands from instructor.py and execute appropriate funtion
    	funcmap = {
    	# variable_from_data : function_to_exectute
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
    	funcmap[data.data]()	# execute

    def interpret_rightarm(self, data):
	# read status updates from right_arm_sim.py and execute appropriate function
        funcmap = {
        'starting_demo': self.begin_sequence,
        'brick_picked': self.movenearcenter,
        'right_at_center': self.takebrick,
        'brick_released': self.placebrick
        }
        funcmap[data.data]()	# execute

    def calibrate(self):
     	# zero the end effector relative to the center of the build area
     	# user should move the end effector to the zero point BEFORE running the calibration function
     	self._cpose = self._limb.endpoint_pose()
     	self._cpose_angles = self._limb.joint_angles()
     	print("Left arm calibrated... frame origin at:\n{}".format(self._cpose))
        self._hover_angles = None
        self.hoverplace()
     	return

    def move_to_start(self):
    	# On initialisation, move to a pre-defined start location
        print('Moving the {0} arm to start pose...'.format(self._limb_name))
        self._guarded_move_to_joint_position(self._start_angles)
        self.gripper_open()

    def move_to_cpose(self):
    	# Move to the recorded calibration pose
        print('Moving the to calibration pose...')
        self._guarded_move_to_joint_position(self._cpose_angles)

    def current_joint_angles(self):
    	# Print current joint angles of left arm
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
        # Convert rospy's string representation of uint8[]'s to int's
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

    def hoverplace(self):
	# Hover the end effector near to the calibration pose: used as a keyframe in placing bricks
    	print("Moving to neutral position by structure")
        if self._hover_angles:
	# if this function has already been run, a set of joint angles for this pose will be stored at a class level
            self._guarded_move_to_joint_position(self._hover_angles)
	    # in this scenario, simply move to the saved joint angles
            return
	# Upon running this function the first time since calibration, use Inverse Kinetmatics to find and move to the hover pose
        calibrationpose = self._cpose
     	neutralpose = Pose()
     	neutralpose.position.x = calibrationpose['position'].x - self._hover_distance		# slightly behind structure
     	neutralpose.position.y = calibrationpose['position'].y
     	neutralpose.position.z = calibrationpose['position'].z + 2.5*self._hover_distance	# some distance above table
        neutralpose.orientation.x = 1
        neutralpose.orientation.y = 1
        neutralpose.orientation.z = 0
        neutralpose.orientation.w = 0
     	joint_angles = self.ik_request(neutralpose)		# use Inverse Kinetmatics to find joint angles
     	self._guarded_move_to_joint_position(joint_angles)
        self._hover_angles = joint_angles			# save joint angles for future function calls

    def placebrick(self, bx = 0.2, by = 0.09, bz=0.062):
    	calibrationpose = self._cpose
        print("Placing brick #{}".format(self._iteration))
    	# collect the brick - iteration determines position
    	brickdict = {
   #brick : [xpos, ypos, zpos, xor, yor, zor, wor]
    	1 : [0.1, -1.80*bx , 0.6*bx     ],               
    	2 : [0.1, -1.35*bx , 0.6*bx     ],     
    	3 : [0.1, -0.90*bx , 0.6*bx     ],          
    	4 : [0.1, -0.45*bx , 0.6*bx     ],          
    	5 : [0.1,  0       , 0.6*bx     ],  
    	6 : [0.1, -1.33*bx , 0.8*bx+bz  ],  
    	7 : [0.1, -0.26*bx , 0.8*bx+bz  ],
    	8 : [0.1, -0.72*bx , 0.8*bx+2.2*bz]
    	}
    	# do not continue if al 8 bricks have been moved
    	if self._iteration > 8:
    		rospy.logerr('Reading all 8 bricks have been placed...')
    		return
        self.hoverplace()
    	brickpose = Pose()
    	# determine brick location based on calibrationpose and brick dictionary
    	brickpose.position.x = calibrationpose['position'].x + brickdict[self._iteration][0]
    	brickpose.position.y = calibrationpose['position'].y + brickdict[self._iteration][1] - 0.1
    	brickpose.position.z = calibrationpose['position'].z + brickdict[self._iteration][2] + 0.2
     	brickpose.orientation.x = 1
        brickpose.orientation.y = 1
        brickpose.orientation.z = 0
        brickpose.orientation.w = 0
    	joint_angles = self.ik_request(brickpose)
    	self._guarded_move_to_joint_position(joint_angles)
        if self._sequence:
            pub.publish('brick_placed')
        brickpose.position.z = calibrationpose['position'].z + brickdict[self._iteration][2]
        joint_angles = self.ik_request(brickpose)
        self._guarded_move_to_joint_position(joint_angles)
    	self._gripper.open()
        brickpose.position.z = calibrationpose['position'].z + brickdict[self._iteration][2] + 0.2
        joint_angles = self.ik_request(brickpose)
        self._guarded_move_to_joint_position(joint_angles)
    	# return to calibration position
    	self.hoverplace()
    	# inform left arm that brick has been taken if running full sequence	

    def movenearcenter(self):
    	# Move arm near central trade position, ready to receive brick from right arm
        centerpose = Pose()
        centerpose.position.x = 0.50	# Position define so that the motion to grab the brick is a straight line
        centerpose.position.y = 0.15
        centerpose.position.z = 0.50
        centerpose.orientation.x = 1
        centerpose.orientation.y = 1
        centerpose.orientation.z = -1
        centerpose.orientation.w = 1
        joint_angles = self.ik_request(centerpose)
        self._guarded_move_to_joint_position(joint_angles)
        self.gripper_open()

	
    def takebrick(self):
        # Take the brick that is currently being held by the right arm
        self._iteration += 1				# Keep track of which brick is currently being grabbed
        if self._iteration in [1, 2, 3, 4, 5]:		# For bricks to be placed vertically
            brickpose = Pose()
            brickpose.position.x = 0.500
            brickpose.position.y = 0.050
            brickpose.position.z = 0.400
            brickpose.orientation.x = 1
            brickpose.orientation.y = 1
            brickpose.orientation.z = -1
            brickpose.orientation.w = 1
            joint_angles = self.ik_request(brickpose)
        elif self._iteration in [6, 7, 8]:		# For bricks to be placed horizontally
            brickpose = Pose()
            brickpose.position.x = 0.500
            brickpose.position.y = 0.000
            brickpose.position.z = 0.500
            brickpose.orientation.x = 1
            brickpose.orientation.y = 1
            brickpose.orientation.z = -1
            brickpose.orientation.w = 1
            joint_angles = self.ik_request(brickpose)
        else:
            # Failsafe in case of user error when testing: this should not be reached during the demo
            ros.logerr('Brick iteration exceeds expected value')
            return
        self._guarded_move_to_joint_position(joint_angles)
        self.gripper_close()
    	if self._sequence:				# Inform right arm that the brick has been grabbed
    		pub.publish('brick_grabbed')

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
	# When user wishes to demo full sequence: reset iteration count and enable communication between arms
    	print("BEGINNING FULL DEMO")
    	self._sequence = True
    	self._iteration = 0


rospy.init_node('left_arm', anonymous=False)		# Create node: only one should be run at once therefore anonymous = False
rospy.wait_for_message('/robot/sim/started', Empty)	# Ensure simulation is running

pub = rospy.Publisher('left_status', String, queue_size=100)	# Topic containing status of the left arm

leftarm = LeftArmControl(verbose=False)			# Initialise the left arm
leftarm.move_to_start()					# Move to pre-defined start position
leftarm.calibrate()					# Initial calibration at start position

def listen():
	# Background function
	rospy.Subscriber('instructor_left', String, leftarm.interpret_instructor)	# Wait for command from instructor file
	rospy.Subscriber('right_status', String, leftarm.interpret_rightarm)		# Or react to right arm status updates
	rospy.spin()									# Listen continuously

while not rospy.is_shutdown():				# Main loop
	print("Left arm running...")
	listen()
