#!/usr/bin/env python

import rospy, sys, numpy as np
import geometry_msgs.msg
import moveit_msgs.msg

from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from time import sleep

def gripper_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper_off():
    rospy.wait_for_service('/ur5/vacuum_gripper/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def main():
    gripper_on()
    sleep(3)
    gripper_off()


# entry point to the program
if __name__ == "__main__":
    
    main()




#try:
#        main()
#    except rospy.ROSInterruptException:
#        pass



