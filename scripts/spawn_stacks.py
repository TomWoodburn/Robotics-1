#!/usr/bin/env python

'''
spawn_stacks.py

Run this file in the background once the BAXTER arms are in their start positions.
This will spawn the initial tables and all necessary brick models in three stacks

These bricks will likely displace themselves in the simulation, rendering the robot unable to manipulate them
This file is primarily included for reference of our methodology: we recommend using spawn_single.py in all simulations
'''

import load_stacks as models        # load functions from load_stacks.py
import rospy

print("Spawning models...")
rospy.init_node("spawn_module")     # create ROS node
models.import_gazebo_models()
models.spawn_block_table()
models.spawn_initial_stack()        # spawn all stacks
print("Spawning complete.")

rospy.on_shutdown(models.delete_gazebo_models)

# this null variable has no purpose except for preventing exit of this script until the user is ready
# when the user presses ENTER, the file will exit and despawn all models with it
null = raw_input("Press ENTER to remove models... ")
