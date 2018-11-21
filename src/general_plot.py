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
import tf
from geometry_msgs.msg import TransformStamped as tfStamped
from geometry_msgs.msg import Transform, PointStamped, Point
from simulation.sim_classes import SimulatedVehicle

visuals = None  # TrackVisuals Object to be used for plotting data
sim_checker = None  # SimilarityChecker Object to be used for comparing objects
transforms = None  # Transformation Array
c2x = None
transformer = None


def callback_tracking(data):
    """
    Plot data from the original laser scans
    """
    global visuals, loc
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


def callback_tf_manual(data):
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
        visuals.plot_points_tuple([next_point], append=True)
        # print("Steps: "+str(steps)+" --- Position: "+str(next_point))  # DEBUG print of the position and # of steps

    steps += 1  # c2x was used in another step, increase the counter
    lock.release()


def callback_tf(data):
    """
    Uses the tf package for the same operation as callback_tf_manual:
    Append a newly acquired transform message to the list of stored transforms, and plot a c2x message if one was
    recorded already.

    TODO currently is just a test for a single object (since it just takes data.transforms[0]. requires generalization
        can probably achieve this by simply adding the setTransform line in the tracks object? (since its 1 tf per car)
    """
    global transforms, steps, c2x, transformer, time_obj
    transforms.append(data)
    tf_object = data.transforms[0]
    # Force time == 0 or you will run into issues TODO time==0 force here
    tf_object.header.stamp = rospy.Time(0)
    transformer.setTransform(tf_object)  # set the data as the transform
    if c2x is None:
        return
    lock.acquire()

    # String frame ids of the source/destination frame
    src_id = "odom"
    dest_id = "ibeo_front_center"

    tracks = c2x.tracks
    seq = c2x.header.seq

    # transformer: tf.TransformerROS
    for track in tracks:
        time_obj = rospy.Time(0)
        x_pos = track.box.center_x
        y_pos = track.box.center_y
        try:
            # Try to transform the point
            # Create a point with z=0 for the source frame (out of c2x tracks x/y) coordinate
            point = PointStamped(header=SimulatedVehicle.create_def_header(frame_id=src_id), point=Point(x=x_pos, y=y_pos, z=0))
            # Don't use the current timestamp, use 0 to use the latest available tf data
            point.header.stamp = rospy.Time(0)
            # Now transform the point using the data
            tf_point = transformer.transformPoint(target_frame=dest_id, ps=point)
            # Print/Visualize the point
            # print("("+str(x_pos)+" | "+str(y_pos)+")  -->  ("+str(tf_point.point.x)+" | "+str(tf_point.point.y)+")")
            next_point = (tf_point.point.x, tf_point.point.y, track.object_id, "y")
            visuals.plot_points_tuple([next_point], append=True)
        except tf.ExtrapolationException as e:
            # Extrapolation error, print but keep going (possible just because only one frame was received so far)
            print(e)
    # End of for going over tracks
    steps += 1  # c2x was used in another step, increase the counter
    lock.release()


def callback_tf_static(data):
    """
    Acquires static tf data and adds it to the transformer object
    """
    global transformer
    for tf_obj in data.transforms:
        # Force time == 0 or you will run into issues TODO time==0 force here
        tf_obj.header.stamp = rospy.Time(0)
        transformer.setTransform(tf_obj)


def listener():
    """
    Prepare the subscribers and setup the plot etc
    """
    global visuals, transformer
    rospy.init_node('listener_laser_scan', anonymous=True)

    transformer = tf.TransformerROS(True)
    # print("FrameList:\t" + transformer.allFramesAsString())
    # Start the subscriber(s)
    rospy.Subscriber("tracked_objects/scan", TrackedLaserScan, callback_tracking)  # General subscriber (tracking data)
    rospy.Subscriber("tf", TFMessage, callback_tf)  # Acquire transform messages
    rospy.Subscriber("/FASCarE_ROS_Interface/car2x_objects", TrackedOrientedBoxArray, callback_c2x)
    rospy.Subscriber("tf_static", TFMessage, callback_tf_static)  # Acquire static transform message for "ibeo" frames
    # rospy.Subscriber("tracked_objects/scan", TrackedLaserScan, callback_org_data)

    if len(sys.argv) > 1:
        fname = sys.argv[1]  # Get the filename
        # now start a rosbag play for that filename
        player_proc = subprocess.Popen(['rosbag', 'play', fname], cwd="data/")

    plt.show()  # DON'T NEED TO SPIN IF YOU HAVE A BLOCKING plt.show

    # Kill the process (if it was started)
    if len(sys.argv)>1:
        player_proc.terminate()


if __name__ == '__main__':
    steps = 0  # how many steps passed since the last time the c2x message was used
    lock = thr.Lock()
    transforms = []
    # Create a new Visualization object with the axis limits and "blue" as default plotting color
    visuals = TrackVisuals(limit=65, neg_limit=-40, limit_y=50, neg_limit_y=-40, color='b')

    listener()
