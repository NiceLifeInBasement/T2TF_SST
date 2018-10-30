#!/usr/bin/env python
"""
Main file for testing the algorithms for creating consistent data from the TrackedLaserScans

Can be given 1 command line argument: name of the bag (in the data/) folder that should be played.
The player will be terminated if the plot window is closed. Still, this causes some screen clutter, so it can be better
to play the bag from a different terminal
"""

import numpy as np
import rospy
from bob_perception_msgs.msg import *
from tracking_visuals import *
import matplotlib.pyplot as plt
from consistency_methods import *
from similarity import *
import subprocess
import sys

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
    if len(sys.argv) > 1:
        fname = sys.argv[1]  # Get the filename
        # now start a rosbag play for that filename
        player_proc = subprocess.Popen(['rosbag', 'play', fname], cwd="data/")

    plt.show()

    # Kill the process (if it was started)
    if len(sys.argv)>1:
        player_proc.terminate()


if __name__ == '__main__':
    # Create a new Visualization object with the axis limits and "blue" as default plotting color
    visuals = TrackVisuals(limit=65, neg_limit=-10, limit_y=30, neg_limit_y=-20, color='b')

    # Check if params for the similarity checker were passed
    if len(sys.argv)>3:  # passed dist_mult and velo_add param
        sim_checker = SimilarityChecker(dist_mult=float(sys.argv[2]), velo_add=float(sys.argv[3]))
    else:
        # No params, create one yourself from hard-coded values
        # Values dist_mult=0.2, velo_add=0 achieve some results (at ttl=4)
        # Both 0 => No detection happening at all

        sim_checker = SimilarityChecker(dist_mult=0.1, velo_add=0.4)
        # Uncomment the following line to display the "normal" data without any consistency checking
        # sim_checker = SimilarityChecker(dist_mult=0, velo_add=0)

    listener()
