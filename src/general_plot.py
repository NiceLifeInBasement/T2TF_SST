#!/usr/bin/env python
"""
General main file that plots data from different sources.
"""

import numpy as np
import rospy
from bob_perception_msgs.msg import *
from tracking_consistency.tracking_visuals import *
import matplotlib.pyplot as plt
from tracking_consistency.similarity import *
import subprocess
import sys
from tf2_msgs.msg import TFMessage
import threading as thr

visuals = None  # TrackVisuals Object to be used for plotting data
sim_checker = None  # SimilarityChecker Object to be used for comparing objects
transforms = None  # Transformation Array
c2x = None


def callback_tracking(data):
    """
    Plot data from the original laser scans
    """
    global visuals, lock
    lock.acquire()
    visuals.plot_box_array(data.boxes, append=False)
    lock.release()


def callback_c2x(data):
    """
    Store the last c2x message
    """
    global transforms, visuals, c2x, steps
    c2x = data
    steps = 0


def callback_tf(data):
    """
    Append a newly acquired transform message to the list of stored transforms, and plot a c2x message if one was
    recorded already.
    """
    global transforms, steps, c2x
    transforms.append(data)
    if c2x is None:
        return
    lock.acquire()

    tracks = c2x.tracks
    seq = c2x.header.seq
    for track in tracks:
        # print("OID: "+str(track.object_id))
        # print("\tseq: "+str(seq))
        # print("\tx: "+str(track.box.center_x))False
        # print("\ty: " + str(track.box.center_y))

        # Subtract the value of the last transform from the value of the current x/y pos
        tf_x = transforms[len(transforms)-1].transforms[0].transform.translation.x
        tf_y = transforms[len(transforms) - 1].transforms[0].transform.translation.y
        x = track.box.center_x
        y = track.box.center_y
        adj_x = x - tf_x
        # Don't know how velocity should be used (is it per second? per timestep? per X? currently acting like it should
        # be used in a per time step/"received msg" form, but that is likely wrong)
        # Since steps==0 often, this doesnt cause too much trouble, but in general this is an issue
        # Could also implement a multiplicative factor 0<f<1 that reduces the impact of velocity
        adj_x += steps * track.box.velocity_x  # Adjust using the last known velocity * number of time steps
        adj_y = y - tf_y
        adj_y += steps * track.box.velocity_y

        # Plot/Print the position of this message
        next_point = (adj_x, adj_y, track.object_id, "y")
        print(str(next_point))
        visuals.plot_points_tuple([next_point], append=True)

    steps += 1  # c2x was used in another step, increase the counter
    lock.release()


def listener():
    """
    Prepare the subscribers and setup the plot etc
    """
    global visuals
    rospy.init_node('listener_laser_scan', anonymous=True)

    # Start the subscriber(s)
    rospy.Subscriber("tracked_objects/scan", TrackedLaserScan, callback_tracking)  # General subscriber (tracking data)
    rospy.Subscriber("tf", TFMessage, callback_tf)  # Acquire transform messages
    rospy.Subscriber("/FASCarE_ROS_Interface/car2x_objects", TrackedOrientedBoxArray, callback_c2x)
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
    steps = 0  # how many steps passed since the last time the c2x message was used
    lock = thr.Lock()
    transforms = []
    # Create a new Visualization object with the axis limits and "blue" as default plotting color
    visuals = TrackVisuals(limit=65, neg_limit=-40, limit_y=50, neg_limit_y=-40, color='b')

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
