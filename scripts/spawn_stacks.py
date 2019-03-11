#!/usr/bin/env python

import models
import rospy

rospy.init_node("spawn_module")

print("Spawning models...")
models.import_gazebo_models()
models.spawn_block_table()
models.spawn_initial_stack()
print("Spawning complete.")

rospy.on_shutdown(models.delete_gazebo_models)

null = raw_input("Press ENTER to remove models... ")
