******************
Sequence Execution
******************

1. First Brick Picked
=====================

The first step in the cascading series of actions that makes up the Sequence is the picking up of the first brick.

When the 'Demo' command is input to the right arm terminal the ``begin_sequence`` function is called. 

.. code-block:: bash
	
		def begin_sequence(self):
    	# When user wishes to demo full sequence: reset iteration count, enable communication between arms, and start demo
    	print("BEGINNING FULL DEMO")
        pub.publish('starting_demo')
    	self._sequence = True
    	self._iteration = 0
    	self.pickbrick()


This ``begin_sequence`` function sets the iteration value to 0, which represents the brick count. It then calls the ``pickbrick`` function, shown in full here: 

.. code-block:: bash
	
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

.. code-block:: bash

		def pickbrick(self, brick_y = 0.09, brick_z = 0.062):
			...
			self._iteration += 1
			self._gripper.open()
			...

Following this the ``brickpose`` - the position the end effector will move to in order to pick up the brick - is set. 

.. code-block:: bash

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

.. code-block:: bash
		
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

.. code-block:: bash
		
		def pickbrick(self, brick_y = 0.09, brick_z = 0.062):
			...
			joint_angles = self.ik_request(brickpose)
			self._guarded_move_to_joint_position(joint_angles)
			self._gripper.close()
			self.hoverbrick()					# Return to neutral hover position
			...


After this the arm returns to the hover pose above the brick using the ``hoverbrick`` function. This is shown below and uses the stored ``calibrationpose`` and combines this with a set hover distance of 0.1 then again uses ``ik_request`` to find the ``_hover_angles`` and then uses ``_guarded_move_to_joint_position`` to move to this position :

.. note:: For the simulation which uses only one brick spawn location and consequently only one hover position, once this function has been run the first time and the ``_hover_angles`` have been created they are re-used and not calculated again as the initial if statement allows this function to be skipped. 

.. code-block:: bash
		
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

.. code-block:: bash
		
		def pickbrick(self, brick_y = 0.09, brick_z = 0.062):
			...
			pub.publish('brick_picked')				# Inform left arm and brick spawner
			if self._sequence:
				self.movetocenter()	



2. Right arm moves to centre
============================

Having set up the gazebo environment and spawned the baxter robot it is then necessary to initialise all our constituent nodes:

.. note:: If running the demo on the physical DE NIRO robot then ignore step 1. If running in simulation follow all steps below

1. Open a new terminal window to spawn the tables and bricks in simulation:

	.. code-block:: bash
	
		cd grasping_ws
		./baxter.sh sim
		rosrun baxter_sim_examples spawn_single.py 

.. note:: If this demo is being run on the physical DE NIRO robot, follow steps 1 through 8. If the demo is being run in simulation then use only steps 4,5 & 12

**Once the demo begins there is no need for further human input, the code operates autonomously using node topics.**

3. Left arm moves to centre
===========================

Having set up the gazebo environment and spawned the baxter robot it is then necessary to initialise all our constituent nodes:

.. note:: If running the demo on the physical DE NIRO robot then ignore step 1. If running in simulation follow all steps below

1. Open a new terminal window to spawn the tables and bricks in simulation:

	.. code-block:: bash
	
		cd grasping_ws
		./baxter.sh sim
		rosrun baxter_sim_examples spawn_single.py 

.. note:: If this demo is being run on the physical DE NIRO robot, follow steps 1 through 8. If the demo is being run in simulation then use only steps 4,5 & 12

**Once the demo begins there is no need for further human input, the code operates autonomously using node topics.**

4. Brick is passed
==================

Having set up the gazebo environment and spawned the baxter robot it is then necessary to initialise all our constituent nodes:

.. note:: If running the demo on the physical DE NIRO robot then ignore step 1. If running in simulation follow all steps below

1. Open a new terminal window to spawn the tables and bricks in simulation:

	.. code-block:: bash
	
		cd grasping_ws
		./baxter.sh sim
		rosrun baxter_sim_examples spawn_single.py 

.. note:: If this demo is being run on the physical DE NIRO robot, follow steps 1 through 8. If the demo is being run in simulation then use only steps 4,5 & 12

**Once the demo begins there is no need for further human input, the code operates autonomously using node topics.**

5. Left arm places brick
========================

Having set up the gazebo environment and spawned the baxter robot it is then necessary to initialise all our constituent nodes:

.. note:: If running the demo on the physical DE NIRO robot then ignore step 1. If running in simulation follow all steps below

1. Open a new terminal window to spawn the tables and bricks in simulation:

	.. code-block:: bash
	
		cd grasping_ws
		./baxter.sh sim
		rosrun baxter_sim_examples spawn_single.py 

.. note:: If this demo is being run on the physical DE NIRO robot, follow steps 1 through 8. If the demo is being run in simulation then use only steps 4,5 & 12

**Once the demo begins there is no need for further human input, the code operates autonomously using node topics.**
