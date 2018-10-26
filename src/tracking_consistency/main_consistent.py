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

visuals = None  # TrackVisuals Object to be used for plotting data


def callback_org_data(data):
    # Plot data that was produced by the original Laser Scans
    global visuals
    box_array = data.boxes
    # Clear lists of points
    ConsistencyTracker.plot_data(box_array, visuals, 'r', False)


def listener():
    global visuals
    rospy.init_node('listener_laser_scan', anonymous=True)

    rospy.Subscriber("tracked_objects/scan", TrackedLaserScan, callback_org_data)

    # create a ConsistencyTracker Object that gets called in the callback function and that plots data in a diff color
    # to the visuals object so that you can compare how it works out
    phi = 0.1
    # cons_track = ConsistencyTracker(working_mode="plot", radius_mult_constant=phi,
    #                                vis=visuals, plot_col='b', append=True)

    # DON'T NEED TO SPIN IF YOU HAVE A BLOCKING plt.show
    plt.show()


if __name__ == '__main__':
    visuals = TrackVisuals(limit=40, neg_limit=-20, color='b')  # Create a new Visualization object
    listener()
