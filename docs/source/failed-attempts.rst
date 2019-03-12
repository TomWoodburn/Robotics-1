*******************************
Simulation Problems Encountered
*******************************


DENIRO Wheel Drift
==================

The addition of wheels onto the Baxter robot to create DENIRO caused the robot to drift from its starting position. This happened as a reaction to the force exerted by the arms such that over the course of a demo, the robot would move and lose calibration. Successive brick pickups and stacks were not possible due to the accumulating error.

Solution
--------

We tried increasing wheel surface friction, with limited results. The final solution for simulator examples was the use of Baxter, rather than DENIRO, robot Environment. Baxter is fixed in place on caster wheels rather than a wheelchair. 


Brick Stack Instability
=======================

An error with either the brick model .urdf and Gazebo simulation meant that bricks stacked on top of each other would slip off when spawned into the environment without an external distrubance, or that they would sink into eachother or the table.

Solution
--------

Setting the table blocks to be static objects aided stability. 

We also tried increasing the frictions value and decreasing the stack size, with little success. A temporary simulation-only fix changed our algorithm to pick up a bricks from the same position each time, replacing the picked up brick with a new brick each cycle. For the physical demonstration the stack system was used. 


Collisions from Inverse Kinematic Solver
========================================

When moving to changeover positions, the inbuilt inverse kinematic solver would sometimes colide with the robots body. It had the right position for the end effector, but the rest of the arm would be in an invalid position for use in real life.

Solution
--------

Manually finding the joint angles for the changeover position. Optimising for minimal joint movement and a position away from the building structure. Moving to fixed joint angles instead of using inverse kinematics. 


End-Effector Gripper too Narrow for Brick
=========================================

The provided urdf file for the DeNiro robot limited the amount the end-effector grippers could open. This resulted in it not being possible to fit around the required size of brick model.

Solution
--------

We edited the urdf file for the end effector, changing the 'open' amount from 3 to 5. This meant that the maximum amount the grippers opened was increased and large enough to fit around the brick.
