#!/usr/bin/env python

'''
spawn_single.py

Run this file in the background once the BAXTER arms are in their start positions.
This will spawn the initial tables and first brick, then will repawns the brick whenever it is picked up by the right arm
'''

import rospy
import load_single as models		# load functions from load_single.py

from std_msgs.msg import String

def respawn(data):			# function to respawn bricks - receives status updates from right arm
    if data.data == 'brick_picked':	# if right arm confirms the brick has been picked up
        print("Spawning brick...")	# update user...
        models.spawn_brick()		# respawn brick in initial position

def listen():						# continuous function: waits for status update from right arm
    rospy.Subscriber('right_status', String, respawn)	# sends received status updates to the respawn function
    rospy.spin()					# continue until shutdown

rospy.on_shutdown(models.delete_gazebo_models)		# when exited, remove all models

print("Initialising spawner...")
rospy.init_node("spawn_module")		# create ROS node
models.import_gazebo_models()		# run preliminary functions from load_single.py
models.spawn_block_table()
models.spawn_brick()
print("Spawner initialised...")		# ready to go

while not rospy.is_shutdown():		# mainloop
	listen()
