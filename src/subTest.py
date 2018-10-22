#!/usr/bin/env python
import rospy
import rostopic
from std_msgs.msg import String
from bob_perception_msgs.msg import *



def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "RECEIVED: %s", data.data)
    rospy.loginfo("Received something")


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)


    # rospy.Subscriber("chatter", String, callback)
    # can use msg type "rospy.msg.AnyMsg" to subscribe to any topic you want, but the resulting data is useless
    #   since it is not deserialized
    # rospy.Subscriber("tracked_objects/scan", rospy.msg.AnyMsg, callback)
    rospy.Subscriber("tracked_objects/scan", TrackedLaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()