*********************
Sequence of Execution
*********************

1. First Brick Picked
=====================

The first step in the cascading series of actions that makes up the Sequence is the picking up of the first brick.

When the 'Demo' command is input to the right arm terminal the ``begin_sequence`` function is called. 

.. code-block:: python
	
        def begin_sequence(self):
            # When user wishes to demo full sequence: reset iteration count, enable communication between arms, and start demo
            print("BEGINNING FULL DEMO")
                pub.publish('starting_demo')
            self._sequence = True
            self._iteration = 0
            self.pickbrick()


This ``begin_sequence`` function sets the iteration value to 0, which represents the brick count. It then calls the ``pickbrick`` function, shown in full here: 

.. code-block:: python
	
		def pickbrick(self, brick_y = 0.09, brick_z = 0.062):
			# Pick up the brick to be manipulated
			calibrationpose = self._cpose
			self._iteration += 1	# Keep track of which brick is being manipulated
			self._gripper.open()
			if self._iteration > 8:			# if all bricks have been picked up
				rospy.logerr('Reading all 8 bricks have been collected...')	# log a console message
				return								# do not pick up another
			print('Collecting brick #{0} from pile'.format(self._iteration))
			brickpose = Pose()
			# determine brick location based on calibrationpose and brick dictionary
			brickpose.position.x = calibrationpose['position'].x
			brickpose.position.y = calibrationpose['position'].y
			brickpose.position.z = calibrationpose['position'].z - brick_z  # one brick height below the calibration pose
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
				self.movetocenter()	

The first order of this function is to increase the iteration (brick count) by 1, then the gripper is open in place using the inbuilt ``_gripper.open`` function.

.. code-block:: python

		def pickbrick(self, brick_y = 0.09, brick_z = 0.062):
			...
			self._iteration += 1
			self._gripper.open()
			...

Following this the ``brickpose`` - the position the end effector will move to in order to pick up the brick - is set. 

.. code-block:: python

		def pickbrick(self, brick_y = 0.09, brick_z = 0.062):
			...
			brickpose = Pose()
			# determine brick location based on calibrationpose and brick dictionary
			brickpose.position.x = calibrationpose['position'].x
			brickpose.position.y = calibrationpose['position'].y
			brickpose.position.z = calibrationpose['position'].z - brick_z  # one brick height below the calibration pose
			brickpose.orientation.x = 0
			brickpose.orientation.y = 1
			brickpose.orientation.z = 0
			brickpose.orientation.w = 0
			...

.. note:: For the digital robot simulation a single brick is continously spawned in one location , meaning the brickpose is unchanged throughout the sequence and is equal to the ``calibrationpose`` minus one brick height ``brick_z``. However, when using the physical DE NIRO robot the bricks start in two stacked piles of 4 and each iteration the brick pose is changed by combining the ``calibrationpose`` and the ``brickdict`` shown below, which is the relative position of a brick to the calibration point using a combination of ``brick_y`` and ``brick_z`` dimensions dependent on the iteration value:

.. code-block:: python
		
		brickdict = {
		# brick number (1-8): position relative to calibration [x, y, z]
		1 : [0, 0	   , -0.7*brick_z],
		2 : [0, 0 	   , -1.7*brick_z],
		3 : [0, 0	   , -2.7*brick_z],
		4 : [0, 0	   , -3.7*brick_z],
		5 : [0, -2*brick_y , -0.7*brick_z],
		6 : [0, -2*brick_y , -1.7*brick_z],
		7 : [0, -2*brick_y , -2.7*brick_z],
		8 : [0, -2*brick_y , -3.7*brick_z]
		}


Once this cartesian ``brickpose`` is defined, the inbuilt inverse kinematics solver is called using ``ik_request`` and the resulting joint angles are input to the ``_guarded_move_to_joint_position`` function which moves the limbs to these joint angles. The gripper is then closed around the brick with ``_gripper.close`` :

.. code-block:: python
		
		def pickbrick(self, brick_y = 0.09, brick_z = 0.062):
			...
			joint_angles = self.ik_request(brickpose)
			self._guarded_move_to_joint_position(joint_angles)
			self._gripper.close()
			self.hoverbrick()					# Return to neutral hover position
			...


After this the arm returns to the hover pose above the brick using the ``hoverbrick`` function. This is shown below and uses the stored ``calibrationpose`` and combines this with a set hover distance of 0.1 then again uses ``ik_request`` to find the ``_hover_angles`` and then uses ``_guarded_move_to_joint_position`` to move to this position :

.. note:: For the simulation which uses only one brick spawn location and consequently only one hover position, once this function has been run the first time and the ``_hover_angles`` have been created they are re-used and not calculated again as the initial if statement allows this function to be skipped. 

.. code-block:: python
		
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
			self._hover_angles = joint_angles

Once the right arm has grabbed the brick and moved upwards to its hover position the final step is to publish to the right arm topic that the brick has been Picked , notifying the left arm. The right arm is then ordered to ``movetocenter``

.. code-block:: python
		
		def pickbrick(self, brick_y = 0.09, brick_z = 0.062):
			...
			pub.publish('brick_picked')				# Inform left arm and brick spawner
			if self._sequence:
				self.movetocenter()	



2. Right arm moves to centre
============================

Having completed the brick pick-up, the Right Arm moves to a 'hover position' above the bricks, defined by stored joint angles which is close to the brick in the centre. Adding this snapshot increases the robustness of the demo. 

Next, the Right Arm moves to a new set of fixed joint angles. This new set of fixed joint angles results in an end-effector pose where the brick is held parallel to the x-axis, and in a central 'passover position' where bricks can be passed to the Left Arm. 

The class ``RightArmControl(object)`` contains the set of hard-coded joint angles for 

::

 1) self._start_angles - joint angles following brickpick
 2) self._h_pass_angles - for when brick are passed horizontally between arms
 3) self._v_pass_angles - for when brick are passed vertically between arms


The 'hover position', the position to which the Right Arm moves to following brick pick up, is defined as a certain distance in the z-axis above the stack of bricks. The distance of the 'hover position' is defined in the class as ``hover_distance``

.. code-block:: python

	class RightArmControl(object):
		def __init__(self, limb='right', hover_distance = 0.25, verbose=True, sequence=False):
			self._limb_name = limb
			self._hover_distance = hover_distance
			self._verbose = verbose				# print debug messages
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


From the 'hover position' the Right Arm moves to the central 'passover position', using the ``movetocenter(self)`` function. 

The first 5 bricks will be placed vertically, therefore ``self.iteration in [1...5]`` link to the ``self._v_pass_angles``

The final 3 bricks will be placed horizontally, therefore ``self.iteration in [6...8]`` link to the ``self._h_pass_angles``

::

 self._guarded_move_to_joint_position(self._h_pass_angles)


``Guarded_move_to`` is a move type that is used so that if problems are encountered in a move,they are logged as errors - preventing code from crashing.

.. code-block:: python

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

Once the brick is moved to centre, The Right Arm, *right_arm.py*, publishes the ``right_at_center`` status topic. The Left Arm is subscribed to this topic, and informs the Left Arm that the brick is in the 'passover position'.


3. Left arm moves to centre
===========================

.. note:: Steps 2 & 3 take place simultaneously

Following on from step 1, once the right arm publishes the string ``'brick_picked'`` to its respective topic, the left arm, which is subscribed to this topic, reads and interprets this command via the ``interpret_rightarm`` function:

.. code-block:: python
		
		def interpret_rightarm(self, data): 
			# read status updates from right_arm_sim.py and execute appropriate function
			funcmap = {
			'starting_demo': self.begin_sequence,
			'brick_picked': self.movenearcenter,
			'right_at_center': self.takebrick,
			'brick_released': self.placebrick
			}
			funcmap[data.data]()	# execute

This string corresponds to a call of the left arms ``movenearcenter`` function, shown in full below:

.. code-block:: python

        def movenearcenter(self):
            # Move arm near central trade position, ready to receive brick from right arm
            centerpose = Pose()
            centerpose.position.x = 0.55	# Position define so that the motion to grab the brick is a straight line
            centerpose.position.y = 0
            centerpose.position.z = 0.32
            centerpose.orientation.x = 1
            centerpose.orientation.y = 1
            centerpose.orientation.z = -1
            centerpose.orientation.w = 1
            joint_angles = self.ik_request(centerpose)
            self._guarded_move_to_joint_position(joint_angles)
            self.gripper_open()

This function has an almost identical form to the right arm equivalent in step 2. The centerpose is a hard coded cartesian position that is fed into the ``ik_request`` solver to provide a set of joint angles and these are then fed into the ``_guarded_move_to_joint_position`` function. With the gripper opened as the final command. 


4. Left arm moves to centre
===========================

Once the left arm sees ``right_at_centre`` on the right arm status topic it calls ``takebrick``

.. code-block:: bash
	
		'right_at_center': self.takebrick,

``Takebrick`` performs the entire action of moving to the brick (to a hover position first) and picking it up.

.. code-block:: python
	
		def takebrick(self):
        		self._iteration += 1
        		if self._iteration in [1, 2, 3, 4, 5]:
            			brickpose = Pose()
            			brickpose.position.x = 0.55
            			brickpose.position.y = -0.15
            			brickpose.position.z = 0.31
            			brickpose.orientation.x = 1
            			brickpose.orientation.y = 1
            			brickpose.orientation.z = -1
            			brickpose.orientation.w = 1
            			joint_angles = self.ik_request(brickpose)

The stored joint angles depend on the required orientation of the brick in the final structure. Bricks 1,2,3,4 and 5 are placed vertically whereas bricks 6,7 and 8 are horizontal. 

.. code-block:: python

		        elif self._iteration in [6, 7, 8]:
            		brickpose = Pose()
           		 	brickpose.position.x = 0.55
                    brickpose.position.y = -0.17
                    brickpose.position.z = 0.35
                    brickpose.orientation.x = 1
                    brickpose.orientation.y = 1
                    brickpose.orientation.z = -1
                    brickpose.orientation.w = 1
                    joint_angles = self.ik_request(brickpose)		

			

Next, ``takebrick`` simply instructs the left arm to follow the final, linear motion to the brick and close it’s gripper. Once it is holding onto the brick, it lets the right arm know so that it can release. 

.. code-block:: python

		if self._sequence:
    			pub.publish('brick_grabbed')


5. Left arm places brick
========================

The left arm always grabs the passover brick using the same orientation. After it has grabbed the brick it needs to place the brick in the correct position. 

To place the brick in its predetermined position, a dictionary of brick positions is used ``brickdict``. 

	.. code-block:: python
	
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

This dictionary gives the position the brick should be placed by using multiples of the brick dimensions (bx, by, bz) and the ``calibrationpos``. This means the structure can be built from any starting home coordinates of the left arm. 

Depending on the iteration number of the brick, the algorithm selects the number of position. The orientation of the end-effector when placing the brick is also dependant on the iteration number. However, this is done through the right arm passing the brick in different orientations. 

Using the calibration pose as the home coordinates for building, and the brick dictionary, the algorithm now has an exact position to place the brick. Using inverse kinematics of the  ``ik_request(self, pose)`` function, it moves to the ``hoverpose``. This moves the left arm to the correct position, but a constant distance ``self._hover_distance`` upwards from it. This ensures that the end-effector does not place the brick with an awkward approach angle or speed. 

Instead, the end-effector can now safely move the brick downwards into position. Having placed the brick in position, the left arm publishes thi to the left arm topic, and releases the brick. 


6. Repeat until the tower is built
==================================

The process of picking up, passing, and placing bricks is looped autonomously until the last brick from the predetermined piles is placed. 

.. figure:: _static/sequence.png
    :align: center
    :figwidth: 30 em
    :figclass: align-center


The instructor, left arm, and right arm, are constantly publishing and listening so that they are aware when a function has been carried out. Once the first brick is placed, the left arm alerts the right to begin the sequence for the next brick. 

The brick dictionary allows for the picking up of bricks from different piles, passing them with alternating orientations, and placing them in relative positions. This loops through until the last brick is placed (the iteration number exceeds 8) and the left arm publishes this. 
