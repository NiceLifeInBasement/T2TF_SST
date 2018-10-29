#!/usr/bin/env python
"""
Main file for testing the algorithms for creating consistent data from the TrackedLaserScans
"""

import numpy as np
import rospy
from bob_perception_msgs.msg import *
from tracking_visuals import *
import matplotlib.pyplot as plt
from consistency_methods import *
from similarity import *

visuals = None  # TrackVisuals Object to be used for plotting data
sim_checker = None  # SimilarityChecker Object to be used for comparing objects


def callback_org_data(data):
    # Plot data that was produced by the original Laser Scans
    global visuals, sim_checker
    box_array = data.boxes
    # Clear lists of points
    ConsistencyTracker.plot_data(box_array, visuals, 'r', False)


def listener():
    global visuals
    rospy.init_node('listener_laser_scan', anonymous=True)

    # Create a ConsistencyTracker Object that will be used for the tracking
    cons_tracker = ConsistencyTracker(sim_checker.sim_position, 0, ttl=4, working_mode="plot", plot_col='b',
                                      vis=visuals, append=False)

    # Start the subscriber(s)
    rospy.Subscriber("tracked_objects/scan", TrackedLaserScan, cons_tracker.callback_consistency)

    # rospy.Subscriber("tracked_objects/scan", TrackedLaserScan, callback_org_data)

    # DON'T NEED TO SPIN IF YOU HAVE A BLOCKING plt.show
    plt.show()


if __name__ == '__main__':
    visuals = TrackVisuals(limit=40, neg_limit=-20, color='b')  # Create a new Visualization object
    sim_checker = SimilarityChecker(dist_mult=0.5, velo_add=0.5)
    listener()
