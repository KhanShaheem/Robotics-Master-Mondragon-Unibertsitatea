#!/usr/bin/python

import group_interface

interface = group_interface.GroupInterface()

print("")
print("MoveIt group: manipulator")
print("Current joint states (radians): {}".format(interface.get_joint_state("manipulator")))
print("Current joint states (degrees): {}".format(interface.get_joint_state("manipulator", degrees=True)))
print("Current cartesian pose: {}".format(interface.get_cartesian_pose("manipulator")))

print("")
print("MoveIt group: endeffector")
print("Current joint states (meters): {}".format(interface.get_joint_state("endeffector")))
print("Current cartesian pose: {}".format(interface.get_cartesian_pose("endeffector")))


print("")
print("Planning group: manipulator")

# HOME POSITION (COMPLETELY HORIZONTAL)
#print("  |-- Reaching named pose...")
#interface.reach_named_pose("manipulator", "home")

# APPROACH GRIPPER TO PIECE
print("  |-- Reaching cartesian pose...")
#pose = interface.get_cartesian_pose("manipulator")
#pose.position.x = 1.20
#pose.position.y = 1.80
#pose.position.z = 0.70
#interface.reach_cartesian_pose("manipulator", pose)

# LIFT THE PIECE WITH GRIPPER
print("  |-- Reaching cartesian pose...")
pose = interface.get_cartesian_pose("manipulator")
pose.position.x = 1.20
pose.position.y = 1.80
pose.position.z = 1.00
interface.reach_cartesian_pose("manipulator", pose)

# JOINT STATE CONTROL (NOT NEEDED)
#print("  |-- Reaching joint state (degrees)...")
#interface.reach_joint_state("manipulator", [0, 0, 30, 0, 0, -20], degrees=True)

# END EFFECTOR CONTROL (NOT NEEDED)
#print("")
#print("Planning group: endeffector")

#print("  |-- Reaching joint state...")
#interface.reach_joint_state("endeffector", [0.025])
