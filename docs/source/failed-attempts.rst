***************
Failed Attempts
***************


DeNiro Simulator Wheel Drift
============================

The additions of wheels onto the baxter robot to create deniro caused the robot to drift from its starting position. This happened either as a reaction to the force exerted by the arms but also due to unforeseen circumstances. This meant over the course of a demo, the robot would move and lose calibration. So successive brick pickups and stacks were not possible.

Solution
--------

For simulator examples we used the Baxter robot Environment. However, for the algorithm to still work we had to change various variables. For example, table height, brick starting position and baxter position. We believe these changes affected the algorithm and caused the real life DeNiro to not work flawlessly.


Brick Stack Simulator Malfunctions
==================================

An error with either the brick model .urdf or the Gazebo simulation, meant that bricks stacked on top of each other would slip off when spawned into the environment without an external distrubance, or that they would sink into eachother or the table.

Solution
--------

Tried increasing the frictions values, this didn't work.
Tried decreasing the stack size, this didn't work.
We changed our algorithm to pick up a bricks from the same position each time. We replace the picked up brick with a new brick each cycle, this does work.


Inaccurate Simulator Inverse Kinematic Solver
=============================================

When programming the changeover positions, the inbuilt inverse kinematic solver would put the arms of the robot through its face. It had the right position for the end effector, but the rest of the arm would be in an invalid position for use in real life.

Solution
--------

Manually finding the joint angles for the changeover position. Optimising for minimal joint movement and a position away from the building structure.


Simulator End-Effector Gripper too Narrow for Brick
===================================================

The provided urdf file for the DeNiro robot limited the amount the end-effector grippers could open. This resulted in it not being possible to fit around the required size of brick model.

Solution
--------

We edited the urdf file for the end effector, changing the 'open' amount from 3 to 5. This meant that the maximum amount the grippers opened was increased and large enough to fit around the brick.
