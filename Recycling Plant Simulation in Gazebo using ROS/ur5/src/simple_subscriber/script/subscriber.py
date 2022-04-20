#!/usr/bin/python

# include packages
import rospy            # client library for ROS 1 (http://wiki.ros.org/rospy)
import std_msgs.msg     # common message types representing primitive data (http://wiki.ros.org/std_msgs)
import sensor_msgs.msg

import cv2
import utils

def callback(message):
    #rospy.loginfo("Message from /topic: %s", message)

    image = utils.image2numpy(message)
    code = cv2.COLOR_BGR2RGB
    image = cv2.cvtColor(image, code)
    cv2.imwrite("image.png", image)

    print("The image has been successfully saved.")

def simple_subscriber():
    # create a subscriber by defining the topic name, the message class and a function to call when data is received (callback)
    sub_topic = rospy.Subscriber("/rrbot/camera1/image_raw", sensor_msgs.msg.Image, callback)
    
    # register client node with the master under the specified name
    rospy.init_node("node_subscriber", anonymous=False)

    # blocks program until ROS node is shutdown
    rospy.spin()


# entry point to the program
if __name__ == "__main__":
    try:
        simple_subscriber()
    except rospy.ROSInterruptException:
        pass
