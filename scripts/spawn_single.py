#!/usr/bin/env python

import rospy
import newmodels

from std_msgs.msg import String

def checker(data):
    if data.data == 'brick_picked':
        print("Spawning brick...")
        newmodels.spawn_brick()

def listen():
    rospy.Subscriber('right_status', String, checker)
    rospy.spin()

rospy.on_shutdown(newmodels.delete_gazebo_models)

print("Initialising spawner...")
rospy.init_node("spawn_module")
newmodels.import_gazebo_models()
newmodels.spawn_block_table()
newmodels.spawn_brick()
print("Spawner initialised...")

while not rospy.is_shutdown():
	listen()
