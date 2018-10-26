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
    global visuals, x, y, sc
    # visuals.clear_plot()
    box_array = data.boxes
    for tracked_box in box_array:
        obj_id = tracked_box.object_id
        oriented_box = tracked_box.box
        x_pos = oriented_box.center_x
        y_pos = oriented_box.center_y
        visuals.plot_point(x_pos, y_pos, 0)

def listen_original_data():
    rospy.init_node('listener_laser_scan', anonymous=True)

    rospy.Subscriber("tracked_objects/scan", TrackedLaserScan, callback_org_data)

    # DONT NEED TO SPIN IF YOU HAVE A BLOCKING plt.show
    plt.show()


if __name__ == '__main__':
    visuals = TrackVisuals()
    print("Listening to org data")
    listen_original_data()
