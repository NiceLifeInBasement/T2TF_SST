#!/usr/bin/env python
import rospy
import rostopic
from std_msgs.msg import String
from bob_perception_msgs.msg import *
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from test import logHelloWorld

# Global Variable Definitions:
uid = 194773  # set to 0 if you want to select a uid from the first data frame


def callback(data):
    # data is the received data, in the format described by the msg file
    box_array = data.boxes  # extract the array of tracked obj boxes

    # Take the first unique object id that you find in the data, and track its x|y across the entire bag

    global uid  # global variable for tracking the uid across all function calls
    if uid == 0:
        # no uid checked for yet
        uid = box_array[2].object_id  # select the tracked uid
    pos = -1  # reset the position counter
    for x in range(0, len(box_array)):
        if box_array[x].object_id == uid:
            pos = x
    if not pos == -1:
        # found the uid in the callback data
        # print its x coordinate
        coord_str = str(box_array[pos].box.center_x) + "|" + str(box_array[pos].box.center_y)
        rospy.loginfo("UID "+str(uid)+" currently has Position: "+coord_str)

        # The following is only necessary to plot the point data
        global ax, fig
        ax.plot(box_array[pos].box.center_x, box_array[pos].box.center_y, 'ro')
        plt.show()
        # Showed the point in the plot
    else:
        rospy.loginfo("UID "+str(uid)+" not found in this callback")


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
    rospy.Subscriber("tracked_objectsFabian/Baum/scan", TrackedLaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()


if __name__ == '__main__':
    logHelloWorld()
    #listener()

    # The following is only necessary to plot the point data
    # Currently all painted dots are shown, this causes overlapping and cluttering of the graph
    global ax, fig
    #fig = plt.figure()
    #ax = plt.axes(xlim=(-10, 10), ylim=(-10, 10))
    #plt.show()
