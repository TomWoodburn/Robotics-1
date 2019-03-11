#!/usr/bin/env python

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

def import_gazebo_models():

    global brick_xml, newbrick_xml, block_table_xml

    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"

    # Load Brick URDF
    brick_xml = '' 
    with open (model_path + "brick/object.urdf", "r") as brick_file:
        brick_xml=brick_file.read().replace('\n', '')

    # Load New Brick SDF
    newbrick_xml = ''
    with open (model_path + "new_brick/model.sdf", "r") as newbrick_file:
        newbrick_xml=newbrick_file.read().replace('\n', '')

    # Load Block Table URDF
    block_table_xml = ''
    with open (model_path + "tables/model.urdf", "r") as block_table_file:
        block_table_xml=block_table_file.read().replace('\n', '')


def spawn_block_table():
    global model_list

    block_table_reference_frame="world"
    block_table1_pose=Pose(position=Point(x=0.5, y=-1.52, z=0.34))
    block_table2_pose=Pose(position=Point(x=0.5, y=-0.22, z=0.34))


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


def spawn_brick_alt(brick_point = Point(x=0.63, y=-0.65, z=0.732)):
    global model_list, iteration

    iteration += 1
    brick_reference_frame="world"
    brick_name = 'Brick' + str(iteration)

    r = quaternion_from_euler(1.57 ,0, 0)
    brick_pose=Pose(position=brick_point,
                    orientation=Quaternion(x=r[0],y=r[1],z=r[2],w=r[3]))

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf(brick_name, newbrick_xml, "/",
                             brick_pose, brick_reference_frame)
        model_list.append(brick_name)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e)) 


def spawn_brick(brick_point = Point(x=0.63, y=-0.65, z=0.732)):
    # ALTERNATIVE BRICK #
    global model_list, iteration

    iteration += 1
    brick_reference_frame="world"
    brick_name = 'Brick' + str(iteration)
   
    r = quaternion_from_euler(0, 0, 1.57)
    brick_pose=Pose(position=brick_point,
                    orientation=Quaternion(x=r[0],y=r[1],z=r[2],w=r[3]))

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf(brick_name, brick_xml, "/",
                             brick_pose, brick_reference_frame)
        model_list.append(brick_name)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))



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

brick_xml = ''
newbrick_xml = ''
block_table_xml = ''
model_list = []
iteration = 0
