#!/usr/bin/env python
from __future__ import print_function
"""
Based on historic_assoc.py

This plays multiple bags (the two bags from the roundabout situation). One bag has an additional delay (the one that 
starts slightly early). Therefore the two bags are played at the same time.

This program sets up subscribers for both files, and converts information from the viewcar2 object to UTM and from 
there into the fixed frame ibeo_front_center of the fascare object.

Afterwards, the data is associated, and results are printed.
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
import general.t2ta_algorithms as t2ta
from tracking_consistency.similarity import *
import os
from general.t2t_history import *
import copy
import tf_conversions as tf_c
import pickle


def callback_fascare(data):
    """
    Callback function for the fascare object.

    In addition to the received data, data from the viewcar2 data buffer is used
    """
    global viewcar2_odom_data, visuals, transformer_fascare
    try:
        viewcar2_odom_data
    except NameError:
        # Alternativly: consider a quick display of the information
        return  # No viewcar2 data was received so far, simply skip this step

    visuals.plot_box_array(data.boxes, append=False, color="b")
    vc_data = viewcar2_odom_data[-1]

    # Transform the last set of viewcar2_data into this frames ibeo_front_center
    src_id = "odom"
    dest_id = "ibeo_front_center"
    tracks = vc_data.boxes
    head = SimulatedVehicle.create_def_header(frame_id=src_id)
    for track in tracks:
        x_pos = track.box.center_x
        y_pos = track.box.center_y
        try:
            point = PointStamped(header=head, point=Point(x=x_pos, y=y_pos, z=0))
            point.header.stamp = rospy.Time(0)
            tf_point = transformer_fascare.transformPoint(target_frame=dest_id, ps=point)
            track.box.center_x = tf_point.point.x
            track.box.center_y = tf_point.point.y

            # TODO currently only transforming x/y position, but nothing else (ignoring velocity etc)

            track.box.header.frame_id = dest_id  # Changed the frame that the point is in
        # For all exceptions: print them, but keep going instead of stopping everything
        except tf.ExtrapolationException as e:
            # Extrapolation error, print but keep going (possible just because only one frame was received so far)
            print(e)
        except tf.ConnectivityException as e:
            # print("Connectivity Exception during transform")
            print(e)
        except tf.LookupException as e:
            # print("Lookup Exception during transform")
            print(e)
    vc_data.header.frame_id = dest_id

    visuals.plot_box_array(vc_data.boxes, append=True, color="y")

    # now to the association:
    # first, add the two tracks to the history
    global history
    history.add("fascare", data.boxes)
    history.add("viewcar2", vc_data.boxes)
    t2tassoc(data.boxes, vc_data.boxes)


def t2tassoc(data_a, data_b):
    """
    Associates the two datasets a and b (in the form of Arrays of trackedOrientedBoxes)
    information is simply printed to the commandline
    :param data_a: The list of trackedorientboxes for the fascare
    :param data_b: The list of trackedorientboxes for the viewcar2
    """

    global t2ta_thresh, hist_size, history, state_space, use_identity

    number_of_associations = 0

    try:
        # generate object ids and sensor names
        fascare_ids = []
        viewcar2_ids = []
        for obj in data_a:
            fascare_ids.append(obj.object_id)
        for obj in data_b:
            viewcar2_ids.append(obj.object_id)
        obj_ids = [fascare_ids, viewcar2_ids]

        sensor_names = ["fascare", "viewcar2"]

        # Data acquisition from the history object should be based on the most recently received objects stamp
        timing = data_a[0].box.header.stamp
        assoc = t2ta.t2ta_historic(obj_ids, sensor_names, t2ta_thresh, hist_size, history, time=timing,
                                   state_space=state_space, use_identity=use_identity)
        ids = []  # this list will hold lists where each entry is an object id in a cluster
        for a in assoc:  # get a list of all associations
            temp = []  # stores ids for one association
            for box in a:  # all tracking boxes in the current association
                temp.append(box.object_id)
            ids.append(temp)  # ids is a list made up of lists of object ids
            if len(a) > 1:
                # If a non-singleton cluster was found, print all ids that belong to it
                print("<<  Non-singleton Cluster: " + str(temp) + "  >>")
                number_of_associations += 1
    # --- except:
    except ValueError as e:
        print("ValueError during association")
        # print(e)
    except IndexError as e:
        print("IndexError during association, likely because not enough data was received yet.")
    print("Finished LIDAR STEP with "+str(number_of_associations)+" associations.")


def callback_viewcar2(data):
    """
    Callback function for the viewcar2 object.
    Received information is transformed to UTM and then saved to a buffer, so that the callback_fascare function
    can process the given information
    """
    # data: TrackedLaserScan
    global transformer_viewcar2

    src_id = "ibeo_front_center"  # Transform data from this...
    dest_id = "odom"  # ... to this. TODO find out which frame this needs to be (odom/gps_antenna?)
    tracks = data.boxes
    head = SimulatedVehicle.create_def_header(frame_id=src_id)
    for track in tracks:
        x_pos = track.box.center_x
        y_pos = track.box.center_y
        try:
            point = PointStamped(header=head, point=Point(x=x_pos, y=y_pos, z=0))
            point.header.stamp = rospy.Time(0)
            tf_point = transformer_viewcar2.transformPoint(target_frame=dest_id, ps=point)
            track.box.center_x = tf_point.point.x
            track.box.center_y = tf_point.point.y

            # TODO currently only transforming x/y position, but nothing else (ignoring velocity etc)

            track.box.header.frame_id = dest_id  # Changed the frame that the point is in
        # For all exceptions: print them, but keep going instead of stopping everything
        except tf.ExtrapolationException as e:
            # Extrapolation error, print but keep going (possible just because only one frame was received so far)
            print(e)
        except tf.ConnectivityException as e:
            # print("Connectivity Exception during transform")
            print(e)
        except tf.LookupException as e:
            # print("Lookup Exception during transform")
            print(e)
    data.header.frame_id = dest_id

    global viewcar2_odom_data
    try:
        viewcar2_odom_data
    except NameError:
        viewcar2_odom_data = []
    viewcar2_odom_data.append(data)


def callback_tf_fascare(data):
    """
    Tf callback for the fascare data
    """
    global transformer_fascare

    for tf_obj in data.transforms:
        # Force time == 0 or you will run into issues TODO time==0 force here
        tf_obj.header.stamp = rospy.Time(0)
        transformer_fascare.setTransform(tf_obj)


def callback_tf_viewcar2(data):
    """
    Tf callback for the viewcar2 data
    """
    global transformer_viewcar2

    for tf_obj in data.transforms:
        # Force time == 0 or you will run into issues TODO time==0 force here
        tf_obj.header.stamp = rospy.Time(0)
        transformer_viewcar2.setTransform(tf_obj)


def listener(start, speed):
    # --- SETUP OF NODE ---
    rospy.init_node('listener_dual_roundabout', anonymous=True)

    rospy.Subscriber("/tracked_objects/scan", TrackedLaserScan, callback_fascare)  # General subscriber (tracking data)
    rospy.Subscriber("/tracked_objects/scan_viewcar2", TrackedLaserScan, callback_viewcar2)

    rospy.Subscriber("/tf", TFMessage, callback_tf_fascare)
    rospy.Subscriber("/tf_viewcar2", TFMessage, callback_tf_viewcar2)

    # Create global variables for the transformers
    global transformer_fascare, transformer_viewcar2
    transformer_viewcar2 = tf.TransformerROS(True)
    transformer_fascare = tf.TransformerROS(True)

    # Don't subscribe to static data, instead:
    # Use pickle to load the stored messages, so that even a starting delay does not cause any issues
    pkl_foldername = "./src/T2TF_SST/data/"  # folder
    pkl_filename = pkl_foldername+"tf_static_dump_fascare.pkl"  # filename
    with open(pkl_filename, 'rb') as pklinput:
        tf_static_data = pickle.load(pklinput)
        callback_tf_fascare(tf_static_data)
    # Do the same again but this time for the viewcar2 static tf data
    pkl_filename = pkl_foldername + "tf_static_dump_viewcar2.pkl"  # filename
    with open(pkl_filename, 'rb') as pklinput:
        tf_static_data = pickle.load(pklinput)
        callback_tf_viewcar2(tf_static_data)

    # Now, start the two processes that play the two bag files

    # viewcar2 starts slightly before fascare
    offset_viewcar2 = 2.58000016212  # THIS VALUE NEEDS TO BE EXACTLY THIS; THIS IS THE DIFF OF START TIMES!

    rate = '-r' + str(speed)
    starttime = '-s ' + str(start)
    starttime_early = '-s ' + str(start+offset_viewcar2)
    FNULL = open(os.devnull, 'w')  # redirect rosbag play output to devnull to suppress it

    # TODO its possible that due to start time or something, this causes them to lose sync before playing

    # Start the fascare player
    fname = "roundabout_fascare_later.bag"
    fascare_proc = subprocess.Popen(['rosbag', 'play', rate, starttime, fname], cwd="data/", stdout=FNULL)
    fname = "roundabout_viewcar2_affix_early.bag"
    viewcar2_proc = subprocess.Popen(['rosbag', 'play', rate, starttime_early, fname], cwd="data/", stdout=FNULL)

    plt.show()  # Start the graphics. This stops execution until the matplotlib window is closed

    # Kill the processes after the matplotlib window was closed
    fascare_proc.terminate()
    viewcar2_proc.terminate()


def setup():
    """
    Setup all necessary variables etc, and start the listener
    """
    # VARIABLE DEFINITIONS
    global start_time, play_rate, t2ta_thresh, hist_size, state_space, use_identity
    start_time = 0
    play_rate = 0.5
    t2ta_thresh = 13
    hist_size = rospy.Duration(0)
    state_space = (True, False, False, False)
    use_identity = True

    # ---
    # FURTHER SETUP
    global visuals
    visuals = TrackVisuals(limit=65, neg_limit=-40, limit_y=50, neg_limit_y=-40, color='b')
    visuals = TrackVisuals(limit=100, neg_limit=-100, color='b')

    global history
    history = TrackingHistory()

    # Start the listener
    listener(start=start_time, speed=play_rate)


if __name__ == '__main__':
    setup()
