**********************
Algorithm Architecture
**********************


Our algorithm is broken down into a number of key nodes:

instructor.py - Used to test the functionality of each arm and run the full demo. In instructor we define two dictionaries that define core functionalities of the arms in terms of messages that can be posted to topics. 

right_arm.py - class for Baxter’s right arm and a set of functions.

left_arm.py - class for Baxter’s left arm and a set of functions.

Spawning - a number of scripts for spawning various objects into the gazebo environment (just for simulation)


Summary of algorithm
====================

Given the nature of our task, we needed to build an algorithm that enables communication between the two arms. We do this using a series of topics which the arms and instructor publish and subscribe to: 

.. figure:: _static/system_architecture.png
    :align: center
    :figwidth: 30 em
    :figclass: align-center


Instructor_left (instructor.py publishing, left_arm.py subscribing)

Instructor_right (instructor.py publishing, right_arm.py subscribing)

Left_status (left_arm.py publishing, right_arm.py subscribing)

Right_status (right_arm.py publishing, left_arm.py subscribing)


Our demo is performed using a sequence of functions which do various parts of the task. These functions work like a ‘cascade’, the last line of each function is calling the next one. Within the functions the topics are listening to and posting to the various topics. We have structured our code in this way because communicating between the arms is critical.