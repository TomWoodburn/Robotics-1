#!/usr/bin/env python

# # # # # # # # # # # # # # # # # # # # # # # # # # #
#                  model_manage.py                  #
#                                                   #
#       Test functionality of individual arms       #
#                  and run full demo                #
#                                                   #
# # # # # # # # # # # # # # # # # # # # # # # # # # #

import rospy
import rospkg
from tf.transformations import *

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


def import_gazebo_models():

    global brick_xml
    global table_xml
    global block_table_xml

    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    # Load Table SDF
    #table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')

    # Load Brick SDF
    brick_xml = ''
    with open (model_path + "new_brick/model.sdf", "r") as brick_file:
        brick_xml=brick_file.read().replace('\n', '')


    # Load Block Table URDF
    block_table_xml = ''
    with open (model_path + "tables/model.urdf", "r") as block_table_file:
        block_table_xml=block_table_file.read().replace('\n', '')


def spawn_table(table_pose=Pose(position=Point(x=1.0, y=0.0, z=-0.3)),
                       table_reference_frame="world"):
    
    global model_list

    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
        model_list.append("cafe_table")
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))



def spawn_block_table():
    global model_list

    block_table_reference_frame="world"
    block_table1_pose=Pose(position=Point(x=0.5, y=-1.22, z=0.34))
    block_table2_pose=Pose(position=Point(x=0.5, y=0.02, z=0.34))


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
    brick_reference_frame="world"
    
    global model_list

    rot = quaternion_from_euler(1.57,0,0)

    brick_pose=Pose(position=brick_point, orientation=Quaternion(x=rot[0],y=rot[1],z=rot[2],w=rot[3]))

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf(brick_name, brick_xml, "/",
                             brick_pose, brick_reference_frame)
        model_list.append(brick_name)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))


def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out

    global model_list
    print('Deleting models')
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        for x in range(len(model_list)):
            resp_delete = delete_model(model_list[x])
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def spawn_initial_stack(x=0.6725, y=-0.5, z=0.73, brick_reference_frame="world"):
    bz = 0.062
    offset = 0.038
    height = z


    for n in range(5):
        print (height+offset)
        position=Point(x, y, height+offset)
        brick_name = 'Brick'+str(n)
        spawn_brick(brick_name, brick_point = position)
        height = height + bz
        rospy.sleep(0.5)

    height = z
    y = y-0.086-0.086

    for n in range(4):
        print (height+offset)
        position=Point(x, y, height+offset)
        brick_name = 'Brick'+str(n+5)
        spawn_brick(brick_name, brick_point = position)
        height = height + bz
        rospy.sleep(0.5)


brick_xml = ''
table_xml = ''
block_table_xml = ''
model_list = []
