#!/usr/bin/env python

'''
load_stacks.py

This file loads in and provides spawning functions for the spawn_stacks.py file.
Spawns two stacks of 3 bricks and one stack of 2 bricks on the rightmost table.

These bricks will likely displace themselves in the simulation, rendering the robot unable to manipulate them
This file is primarily included for reference of our methodology: we recommend using load_single.py in all simulations
'''

import rospy
import rospkg
from tf.transformations import quaternion_from_euler

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

brick_xml = ''          # location of brick file (to be updated in function)
table_xml = ''          # location of new brick file (to be updated in function)
block_table_xml = ''    # location of table file (to be updated in function)
model_list = []         # list of spawned models, allows for easy deletion


def import_gazebo_models():
    global brick_xml, table_xml, block_table_xml    # import global variables for editing

    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"

    # Load Brick URDF
    with open (model_path + "brick/object.urdf", "r") as brick_file:
        brick_xml=brick_file.read().replace('\n', '')

    # Load New Brick SDF (alternative file)
    with open (model_path + "new_brick/model.sdf", "r") as newbrick_file:
        newbrick_xml=newbrick_file.read().replace('\n', '')

    # Load Block Table URDF
    block_table_xml = ''
    with open (model_path + "tables/model.urdf", "r") as block_table_file:
        block_table_xml=block_table_file.read().replace('\n', '')


def spawn_block_table():
    # Function to spawn two solid tables of the correct height in front of the BAXTER robot
    global model_list
    block_table_reference_frame="world"
    block_table1_pose=Pose(position=Point(x=0.5, y=-1.22, z=0.34))  # rightmost table
    block_table2_pose=Pose(position=Point(x=0.5, y=0.02, z=0.34))   # leftmost table

    # Spawn Block table URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block_table1", block_table_xml, "/",
                               block_table1_pose, block_table_reference_frame)
        model_list.append("block_table1")

        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block_table2", block_table_xml, "/",
                               block_table2_pose, block_table_reference_frame)
        model_list.append("block_table2")

    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))


def spawn_brick(brick_name, brick_point = Point(x=0.6725, y=0.1265, z=0.5)):
    # Function to spawn an individual brick - called by the spawn_initial_stack function
    global model_list
    rot = quaternion_from_euler(0,0,1.57)   # Rotate brick model to correct orientation
    
    brick_reference_frame="world"
    brick_pose=Pose(position=brick_point,
                    orientation=Quaternion(x=rot[0],y=rot[1],z=rot[2],w=rot[3]))

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf(brick_name, brick_xml, "/",
                             brick_pose, brick_reference_frame)
        model_list.append(brick_name)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))


def spawn_initial_stack(x=0.6725, y=-0.4, z=0.73, brick_reference_frame="world"):
    # Function to spawn three stacks of bricks when the function is run.
    
    bz = 0.062                                  # Brick thickness
    by = 0.086                                  # Brick width
    offset = 0.008                              # Buffer gap to prevent clipping
    height = z
    for n in range(3):                          # First stack of three bricks
        position=Point(x, y, height+offset)
        brick_name = 'Brick'+str(n)             # Bricks must have unique names
        spawn_brick(brick_name, brick_point = position)
        height = height + bz                    # Update height for next brick (one thickness above previous)
        rospy.sleep(0.5)

    height = z                                  # Reset height to table level
    y  -= 2*by                                  # Next stack is two brick widths to the right
    for n in range(3):                          # Second stack of three bricks
        print (height+offset)
        position=Point(x, y, height+offset)
        brick_name = 'Brick'+str(n+3)
        spawn_brick(brick_name, brick_point = position)
        height = height + bz
        rospy.sleep(0.5)

    height = z                                  # Reset height to table level
    y -= 2*by                                   # Final stack is another two brick widths to the right
    for n in range(2):                          # Last stack contains only two bricks
        print (height+offset)
        position=Point(x, y, height+offset)
        brick_name = 'Brick'+str(n+6)
        spawn_brick(brick_name, brick_point = position)
        height = height + bz
        rospy.sleep(0.5)


def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    global model_list
    print('Deleting models')
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        for x in range(len(model_list)):
            resp_delete = delete_model(model_list[x])
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))
