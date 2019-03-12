***********************************
Initialising Nodes & Beginning Demo
***********************************

Initialising Arms & Calibration
===============================

Having set up the gazebo environment and spawned the baxter robot it is then necessary to initialise all our constituent nodes:

.. note:: If running the demo on the physical DE NIRO robot then ignore step 1. If running in simulation follow all steps below

1. Open a new terminal window to spawn the tables and bricks in simulation:

	.. code-block:: bash
	
		cd grasping_ws
		./baxter.sh sim
		rosrun baxter_sim_examples spawn_single.py 

2. Open another terminal window to initialise the left arm node:

	.. code-block:: bash
	
		cd grasping_ws
		./baxter.sh sim
		rosrun baxter_sim_examples left_arm.py 

3. Open another terminal window to initialise the right arm node:

	.. code-block:: bash
	
		cd grasping_ws
		./baxter.sh sim
		rosrun baxter_sim_examples right_arm.py 

Initialising Instructor and Beginning Demo
==========================================

Once the left_arm, right_arm and spawn_single nodes have been initialised in separate terminal windows , the final node to initialise is the instructor. However, unlike the other nodes, Instructor is setup to run as an anonymous node. This allows it to be initialised multiple times, which is necessary to calibrate both arms of the robot in physical space.

.. note:: If this demo is being run on the physical DE NIRO robot, follow steps 1 through 8. If the demo is being run in simulation then use only steps 4,5 & 12

4. Open a new terminal window to Initialise the first instructor node:

	.. code-block:: bash
	
		cd grasping_ws
		./baxter.sh sim
		rosrun baxter_sim_examples instructor.py 

5. When prompted to select an arm, input '2' for right arm.

	.. code-block:: bash
	
		SELECT ARM: ('1' = left '2' = right)
		2

6. Physically manipulate DE NIRO’s right arm to place it directly above the stack of building bricks.

7. In the same terminal window, input ‘1’ for calibration and press enter. This will store DE NIRO’s joint angles for later reference when picking up bricks 

	.. code-block:: bash
	
		ENTER COMMAND: 
		1

8. Repeat step 1:

	.. code-block:: bash
	
		cd grasping_ws
		./baxter.sh sim
		rosrun baxter_sim_examples right_arm.py 

9. In this instructor Terminal window input ‘1’ to select the left arm.

	.. code-block:: bash
	
		SELECT ARM: ('1' = left '2' = right)
		1

10. Manipulate DE NIRO’s left arm to be positioned at the centre of the target table, touching the surface. 

11. Input ‘1’ into the terminal window to calibrate for this position and store the joint angles. This position acts as a reference with the tower being built relative to this point.

	.. code-block:: bash
	
		ENTER COMMAND: 
		1

12. In the 1st (right arm) terminal when prompted to input another command , enter ‘2’ to begin the complete demo.

	.. code-block:: bash
	
		ENTER COMMAND: 
		2

**Once the demo begins there is no need for further human input, the code operates autonomously using node topics.**
