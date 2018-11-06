#!/usr/bin/env python
# This file performs some bag_analysis that is not easy to do via rosbag/rqt_bag

# Currently using this to analyse the v2x communication topic

import rospy
import rostopic
from std_msgs.msg import String
from bob_perception_msgs.msg import *

# Globals
count = 0
ids = set()
# Init the min and max trackers
min_x = sys.maxsize
min_y = min_x
max_x = (-1)*min_x
max_y = max_x
do_min_max_track = False

dtype = rospy.msg.AnyMsg  # Default beginning data type to prevent errors, should be changed later in all cases


def callback(data):
    global count, ids, min_x, min_y, max_x, max_y, do_min_max_track
    # analysis of the next batch of data
    try:
        track_array = data.tracks
    except AttributeError:
        track_array = data.boxes
        # currently only supports msg type TrackedLaserScan (and TrackedOrientedBoxArray as default)
        # due to the way the boxes are accessed in the data
    for entry in track_array:
        if entry.object_id not in ids:
            print("Added " + str(entry.object_id) + " to the set, total size is now: "+str((len(ids)+1)))
        ids.add(entry.object_id)
        if do_min_max_track:
            x = entry.box.center_x
            y = entry.box.center_y
            if x < min_x:
                min_x = x
                print("New MIN_X: " + str(min_x))
            if y < min_y:
                min_y = y
                print("New MIN_Y: " + str(min_y))
            if x > max_x:
                max_x = x
                print("New MAX_X: " + str(max_x))
            if y > max_y:
                max_y = y
                print("New MAX_Y: " + str(max_y))


def listener(topic_name, data_type):
    """
    Creates the listener object and subscribes to the topic
    :param topic_name: name of the topic to subscribe to
    :param data_type: name of the msg type of the topic
    :return:
    """
    # Init the node with a unique nameTrackedOrientedBoxArray
    rospy.init_node('listener', anonymous=True)

    # Subscribe to the topic of interest
    # need to use eval because the function requires a class to be passed (instead of a string)
    rospy.Subscriber(topic_name, eval(data_type), callback)

    # Prevent python from exiting
    rospy.spin()


if __name__ == '__main__':
    if len(sys.argv)<3:
        # Nothing special was defined, look at the c2x data
        topic = "FASCarE_ROS_Interface/car2x_objects"
        dtype = "TrackedOrientedBoxArray"
    else:
        topic = sys.argv[1]
        dtype = sys.argv[2]
    listener(topic, dtype)
