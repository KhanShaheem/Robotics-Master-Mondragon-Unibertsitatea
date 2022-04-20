#!/usr/bin/python2

# include packages
import rospy            # client library for ROS 1 (http://wiki.ros.org/rospy)
import std_msgs.msg     # common message types representing primitive data (http://wiki.ros.org/std_msgs)


def simple_publisher():
    # create a publisher by defining a topic name, a message class and the queue size used for asynchronously publishing messages
    pub_topic = rospy.Publisher("/topic", std_msgs.msg.String, queue_size=10)
    
    # register client node with the master under the specified name
    rospy.init_node("node_publisher", anonymous=False)
    rate = rospy.Rate(10)       # convenience instance for sleeping in a loop at a specified rate (Hz)

    # message instance
    message = std_msgs.msg.String()
    message.data = "My first ROS topic"     # set the content of the message
    
    # start a loop
    while not rospy.is_shutdown():
        pub_topic.publish(message)  # publish the message to this topic
        rate.sleep()                # sleep at the specified rate (Hz)


# entry point to the program
if __name__ == "__main__":
    try:
        simple_publisher()
    except rospy.ROSInterruptException:
        pass
