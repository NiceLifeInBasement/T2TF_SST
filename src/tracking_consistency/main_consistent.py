#!/usr/bin/env python
"""
Main file for testing the algorithms for creating consistent data from the TrackedLaserScans
"""

import numpy as np
import rospy
from bob_perception_msgs.msg import *
from tracking_visuals import *
import matplotlib.pyplot as plt

visuals = None

x = []
y = []


def callback_org_data(data):
    global visuals
    box_array = data.boxes
    # Clear lists of points
    x_pos = []
    y_pos = []
    uid = []
    for tracked_box in box_array:
        # Extract relevant information from every box (id, position)
        obj_id = tracked_box.object_id
        oriented_box = tracked_box.box
        # Append this information to the list of points for this step
        x_pos.append(oriented_box.center_x)
        y_pos.append(oriented_box.center_y)
        uid.append(obj_id)

    # Update the plot with the list of points from this plot
    colors = np.repeat(['r', 'b', 'g'], len(uid))  # Testing colors
    visuals.plot_points(x_pos, y_pos, uid, colors)


def listen_original_data():
    rospy.init_node('listener_laser_scan', anonymous=True)

    rospy.Subscriber("tracked_objects/scan", TrackedLaserScan, callback_org_data)

    # DON'T NEED TO SPIN IF YOU HAVE A BLOCKING plt.show
    plt.show()


if __name__ == '__main__':
    visuals = TrackVisuals(limit=50, neg_limit=-50, color='b')  # Create a new Visualization object
    listen_original_data()