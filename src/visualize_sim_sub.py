#!/usr/bin/env python
"""
Subscribes to topics that contain information about the simulated data and visualizes incoming data.
"""
import rospy
from tracking_consistency.tracking_visuals import *
from bob_perception_msgs.msg import TrackedOrientedBoxArray
import threading as thr


def callback_measurement(data):
    # Visualize measured data (noisy) in red
    # return
    global vis, vis_lock
    boxes = data.tracks
    vis_lock.acquire()
    vis.plot_box_array(boxes, color='r', append=True)
    vis_lock.release()


def callback_fused(data):
    # Visualize data from a simple fusing algorithm in green
    global vis, vis_lock
    boxes = data.tracks
    vis_lock.acquire()
    vis.plot_box_array(boxes, color='g', append=True)
    vis_lock.release()


def callback_CI(data):
    # Visualize data from the Covariance Intersection algorithm in yellow
    global vis, vis_lock
    boxes = data.tracks
    vis_lock.acquire()
    vis.plot_box_array(boxes, color='y', append=True)
    vis_lock.release()


def callback_clear(data):
    # Visualize the ground truth in blue
    global vis, vis_lock
    boxes = data.tracks
    vis_lock.acquire()
    vis.plot_box_array(boxes, color='b', append=False)
    vis_lock.release()


def subscriber(no_measurements):
    rospy.init_node("simulation_listener", anonymous=True)
    # Set up the subscribers for the ground truth and measured data
    # Truth-sub will clear the data on plot
    rospy.Subscriber("/t2t_sim/truth", TrackedOrientedBoxArray, callback_clear)
    for i in range(no_measurements):
        tname = "/t2t_sim/measured_"+str(i)
        # measurement-subs will append to the data instead of clearing it
        rospy.Subscriber(tname, TrackedOrientedBoxArray, callback_measurement)

    rospy.Subscriber("/t2t_sim/fused", TrackedOrientedBoxArray, callback_fused)
    rospy.Subscriber("/t2t_sim/fused_CI", TrackedOrientedBoxArray, callback_CI)

    plt.show()


if __name__ == '__main__':
    limit = 40
    measure = 2
    vis = TrackVisuals(limit=limit, neg_limit=-limit, limit_y=limit*2, neg_limit_y=-limit*2)
    vis_lock = thr.Lock()
    subscriber(measure)

