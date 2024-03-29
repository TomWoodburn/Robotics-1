**********************************************
DE3-ROB1 Multi Arm Construction Documentation
**********************************************

About
=====

This is the documentation for the group Multi Arm Construction Project for the Robotics 1 module in Design Engineering, Imperial College London, March 2019.

The project is hosted on GitHub: https://github.com/bencobley/Robotics-1.

Authors
-------

* Ben Cobley
* James Krasucki
* Joseph Shepherd
* Matthew Hamiltion
* Robert Hyde
* Thomas Woodburn
* Nirav Ganju-Cass


Summary
=======

The goal of this project was to create a fully automated robot that can pass bricks from one hand to the other by applying code to the DeNiro Robot. It should achieve this at a speed faster than a child could perform the task. The project was written in Python and ROS was used to interface with the modified Baxter robot.

The project includes:

* **Forward and Inverse Kinematics**: To use the 7-dof kinematic model for Cartesian-to-Joint space mapping.
* **Redundancy Resolution**: To resolve the 7-dof arm configuration to minimise excessive joint movements.
* **Motion Planning**: Key positions recorded to ensure safe motion between poses.
* **End-Effector control**: To control the grasping with the 2 fingers of the hand.

Future improvements to the project could include a final implementation of automated calibration as well as a smoother, more continuous trajectory. Additionally, the structure the robot picks up from and builds can be optimised. Overall, the main aims of this project were successfully achieved. The robot is faster than a child at picking up, swapping hands and placing the bricks in a structure.

.. figure:: _static/passover.jpg
    :align: center
    :figwidth: 40 em
    :figclass: align-center

Documentation
=============

This documentation covers the installation, running and operation of the Multi Arm Construction project code with DeNiro. For further questions or troubleshooting contact the Authors above. 


.. toctree::
   :maxdepth: 2
   :caption: Setup and operation

   virtualmachine
   calibration
   initialisation
   videos

.. toctree::
   :maxdepth: 2
   :caption: Algorithm Development

   architecture
   sequence
   simulationproblems

.. toctree::
   :maxdepth: 2
   :caption: Appendices

   readthedocs

