#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


def logHelloWorld():
    # Implementing this for testing purposes:
    #   - Test if ros/this node is running and working
    #   - Test connection to other nodes (importing this function etc)
    rospy.loginfo("Hello World - by test.py - using loginfo")
    print("Hello World - by test.py - using print")


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
