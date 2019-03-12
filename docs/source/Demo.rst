**************
Demo Execution
**************

1. First Brick Picked
=====================

The first step in the cascading series of actions that makes up the demo is the picking up of the first brick.

When the demo command is input to the right arm terminal the ''begin_sequence'' function is called via the ''interpret_instructor'' function - Link to section - 

.. code-block:: bash
	
		def begin_sequence(self):
    		print("BEGINNING FULL DEMO")
    		self._sequence = True
    		self._iteration = 0
    		pickbrick()

This ''begin_sequence'' function sets the iteration value to 0, which represents the brick count. It then calls the ''pickbrick'' function. 

.. code-block:: bash
	def pickbrick(self, calibrationpose=None, brick_y = 0.09, brick_z = 0.062):
			if calibrationpose is None:
				calibrationpose = self._cpose
			# update number of next brick to take
			self._iteration += 1
			# collect the brick - iteration determines position
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
			self._gripper.open()
			rospy.sleep(1.0)
			# do not continue if al 8 bricks have been moved
			if self._iteration > 8:
				rospy.logerr('Reading all 8 bricks have been collected...')
				return
			# move above second stack if second group of 4 bricks
			print('Collecting brick #{0} from pile'.format(self._iteration))
			if self._iteration > 4:
				hoverpose = Pose()
				hoverpose.position.x = calibrationpose['position'].x
				hoverpose.position.y = calibrationpose['position'].y - 2*brick_y
				hoverpose.position.z = calibrationpose['position'].z
				hoverpose.orientation.x = 0
				hoverpose.orientation.y = 1
				hoverpose.orientation.z = 0
				hoverpose.orientation.w = 0
				joint_angles = self.ik_request(hoverpose)
				self._guarded_move_to_joint_position(joint_angles)
			brickpose = Pose()
			# determine brick location based on calibrationpose and brick dictionary
			brickpose.position.x = calibrationpose['position'].x + brickdict[self._iteration][0]
			brickpose.position.y = calibrationpose['position'].y + brickdict[self._iteration][1]
			brickpose.position.z = calibrationpose['position'].z + brickdict[self._iteration][2]
			brickpose.orientation.x = 0
			brickpose.orientation.y = 1
			brickpose.orientation.z = 0
			brickpose.orientation.w = 0
			joint_angles = self.ik_request(brickpose)
			self._guarded_move_to_joint_position(joint_angles)
			self._gripper.close()
			. . . . 

The first order of this function is to increase the iteration (brick count) by 1, then the gripper is open in place using the ''gripper.open'' function

.. code-block:: bash
	self._iteration += 1
	. . . . 
	self._gripper.open()

Following this the ''brickpose'' , the position the end effector will move to to pick up the brick 

In the previous section - Initisation - we calibrated the start position of the right arm, directly above the first brick to be picked. this was stored as the ''cpose'' variable.

.. note:: If running the demo on the physical DE NIRO robot then ignore step 1. If running in simulation follow all steps below

1. Open a new terminal window to spawn the tables and bricks in simulation:

	.. code-block:: bash
	
		def begin_sequence(self):
    		print("BEGINNING FULL DEMO")
    		self._sequence = True
    		self._iteration = 0
    		pickbrick()

.. note:: If this demo is being run on the physical DE NIRO robot, follow steps 1 through 8. If the demo is being run in simulation then use only steps 4,5 & 12

**Once the demo begins there is no need for further human input, the code operates autonomously using node topics.**

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