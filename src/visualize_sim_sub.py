#!/usr/bin/env python
"""
Subscribes to topics that contain information about the simulated data and visualizes incoming data.
"""
import rospy
from tracking_consistency.tracking_visuals import *
from bob_perception_msgs.msg import TrackedOrientedBoxArray


def callback_append(data):
    global vis
    boxes = data.tracks
    vis.plot_box_array(boxes, color='r', append=False)


def callback_clear(data):
    global vis
    boxes = data.tracks
    vis.plot_box_array(boxes, color='b', append=False)


def subscriber(no_measurements):
    rospy.init_node("simulation_listener", anonymous=True)
    # Set up the subscribers for the ground truth and measured data
    # Truth-sub will clear the data on plot
    rospy.Subscriber("/t2t_sim/truth", TrackedOrientedBoxArray, callback_clear)
    for i in range(no_measurements):
        tname = "/t2t_sim/measured_"+str(i)
        # measurement-subs will append to the data instead of clearing it
        rospy.Subscriber(tname, TrackedOrientedBoxArray, callback_append)

    plt.show()


if __name__ == '__main__':
    limit = 80
    measure = 2
    global vis
    vis = TrackVisuals(limit=limit, neg_limit=-limit)
    subscriber(2)

