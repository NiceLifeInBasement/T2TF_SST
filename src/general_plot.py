#!/usr/bin/env python
"""
General main file that plots data from different sources.

In the listener function, select which callback you want for the tf data.
The setup that works best so far is:
    Use the callback_tf_static function for both tf subscribers (/tf and /tf_static).
        This lets them simply add all their tfs to the tf object
    Use the callback_c2x_tf function for the c2x subscriber
        This function transforms the data with the most recently received tf data and then stores it in a list
    Use the callback_tracking function for the tracker
        Set inc_c2x to True, since none of the above functions plots the c2x data.
        Set reset_tf to False, since the transform was done before storing c2x data
        This function will then pick the c2x data that matches the timing of the lidar track and display it.
        Since the c2x data that will be picked was already transformed with the tf data that was available when it was
        received, it will match the lidar data

If you want to plot everything from the callback_tracking, use callback_tf_append, if you dont want that use callback_tf
and callback_tf_static
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

visuals = None  # TrackVisuals Object to be used for plotting data
sim_checker = None  # SimilarityChecker Object to be used for comparing objects
transforms = None  # Transformation Array
c2x = []
transformer = None
tf_list = []
src_id = "odom"
dest_id = "ibeo_front_center"


def closest_match(data, stamp):
    """
    Selects the closest match wrt time from the list data compared to the timestamp stamp
    :param data: A list of objects with a header that have a time stamp
    :param stamp: Timestamp to compare to
    :return: The object from the list with the closest matching time stamp
    """
    if data is None or stamp is None or len(data) == 0:
        return None  # return None if no data was given
    min_val = rospy.Duration(99999999, 0)  # init value is a very long duration, so that the first comp is smaller
    min_pos = 0  # position of the minimum
    for i in range(len(data)):  # use a counter var to store min position
        datapoint = data[i]  # select the current data point from the list
        diff = datapoint.header.stamp - stamp
        diff.secs = abs(diff.secs)
        diff.nsecs = abs(diff.nsecs)
        if diff < min_val:
            min_val = diff
            min_pos = i
    # print("selected"+str(min_pos)+"/"+str(len(data))+" data with diff: "+
    #      str(min_val.secs)+" nsecs:"+str(min_val.nsecs))
    return data[min_pos]


def callback_tracking(data):
    """
    Plot data from the original laser scans
    """
    inc_c2x = True  # Flag to determine whether this should include c2x data
    reset_tf = False  # Flag to determine whether this should reset c2x+ibeo_front_center tf data from tf_list
    global visuals, lock, transforms, steps, c2x
    lock.acquire()
    # c2x_selection = c2x_list[len(c2x_list)-1]  # Select the last c2x data
    # Don't take the last received c2x, but instead take the c2x data that is closest to the current data (wrt time)
    c2x_selection = closest_match(c2x, data.header.stamp)
    visuals.plot_box_array(data.boxes, append=False)
    if reset_tf:
        # Acquire a set of all names of tfs
        all_names = set()
        for tf_obj in tf_list:
            all_names.add(tf_obj.header.frame_id)
        transformer.clear()
        for frame_name in all_names:
            # Create a sublist that contains all tf data with frame_id == frame_name
            frame_list = []
            for tf_obj in tf_list:
                if tf_obj.header.frame_id == frame_name:
                    # object matches frame_id name
                    frame_list.append(tf_obj)
            # frame_list now contains all relevant objects for the current frame_name
            # now determine the closest match of tf data from this list
            best_match = closest_match(frame_list, data.header.stamp)
            best_match.header.stamp = rospy.Time(0)
            transformer.setTransform(best_match)

    if inc_c2x and c2x_selection is not None:
        # Also include c2x data in the plot

        tracks = c2x_selection.tracks
        # transformer: tf.TransformerROS
        for track in tracks:
            time_obj = rospy.Time(0)
            x_pos = track.box.center_x
            y_pos = track.box.center_y
            # Experimental transform of box l/w, velocity
            x_vel = track.box.velocity_x
            y_vel = track.box.velocity_y
            length = track.box.length
            width = track.box.width
            try:
                # Try to transform the point
                # Create a point_pos with z=0 for the source frame (out of c2x tracks x/y) coordinate
                src_f_id = c2x_selection.header.frame_id  # Don't use the global var, instead use the set value
                head = SimulatedVehicle.create_def_header(frame_id=src_f_id)
                # Don't use the current timestamp, use 0 to use the latest available tf data
                head.stamp = rospy.Time(0)
                # The above line allows for tf outside this function and prevents performing multiple tf steps
                point_pos = PointStamped(header=head, point=Point(x=x_pos, y=y_pos, z=0))
                point_vel = PointStamped(header=head, point=Point(x=x_vel, y=y_vel, z=0))
                # check x/y for the length/width point? might depend on angle and not be this easy
                point_lw = PointStamped(header=head, point=Point(x=width, y=length, z=0))
                # Now transform the point using the data
                tf_point_pos = transformer.transformPoint(target_frame=dest_id, ps=point_pos)
                tf_point_vel = transformer.transformPoint(target_frame=dest_id, ps=point_vel)
                tf_point_lw = transformer.transformPoint(target_frame=dest_id, ps=point_lw)

                # Store the results back in the track for further use
                track.box.center_x = tf_point_pos.point.x
                track.box.center_y = tf_point_pos.point.y
                track.box.velocity_x = tf_point_vel.point.x
                track.box.velocity_y = tf_point_vel.point.y
                # For the following 2, check x/y and if tf them like this makes sense at all
                track.box.length = tf_point_lw.point.y
                track.box.width = tf_point_lw.point.x

                # Plotting of the newly transformed point
                next_point = (tf_point_pos.point.x, tf_point_pos.point.y, track.object_id, "y")
                visuals.plot_points_tuple([next_point], append=True)
            except tf.ExtrapolationException as e:
                # Extrapolation error, print but keep going (possible just because only one frame was received so far)
                print(e)
            except tf.ConnectivityException as e:
                # print("Connectivity Exception during transform")
                print(e)
        # End of for going over tracks
        steps += 1  # c2x was used in another step, increase the counter

        # Attempt to perform T2TA here
        sensor_tracks = []
        sensor_tracks.append(data.boxes)  # one sensor track is the data for this callback (=lidar tracking data)
        sensor_tracks.append(tracks)  # The other one is the c2x tracking data

        try:
            assoc = t2ta.t2ta_collected(sensor_tracks, threshold=t2ta_thresh, distance=sim_fct)
            # analyse the results
            ids = []  # this list will hold lists where each entry is an object id in a cluster
            for a in assoc:  # get a list of all associations
                temp = []  # stores ids for one association
                for box in a:  # all tracking boxes in the current association
                    temp.append(box.object_id)
                ids.append(temp)  # ids is a list made up of lists of object ids
            for cl_ids in ids:
                if 100 in cl_ids:
                    if len(cl_ids) > 1:
                        print("<<  Non-singleton Cluster containing 100: "+str(cl_ids)+"  >>")
                    else:
                        print("<<  Singleton Cluster containing 100 found  >>")
        except ValueError:
            print("ValueError during t2ta")
        # --- debug:
        # print(str(data.header.stamp.secs)+" vs. "+str(c2x_selection.header.stamp.secs)+
        #      " --diff: "+str(np.abs(data.header.stamp.secs-c2x_selection.header.stamp.secs))+" !!!")

    # Finished all plotting, release the lock
    lock.release()


def callback_c2x(data):
    """
    Store the last c2x message
    """
    global transforms, visuals, c2x, steps
    c2x.append(data)
    steps = 0


def callback_c2x_tf(data):
    """
    Stores the last c2x message, but transforms it beforehand
    """
    tracks = data.tracks
    # transformer: tf.TransformerROS
    for track in tracks:
        time_obj = rospy.Time(0)
        x_pos = track.box.center_x
        y_pos = track.box.center_y
        try:
            # Try to transform the point
            # Create a point with z=0 for the source frame (out of c2x tracks x/y) coordinate
            point = PointStamped(header=SimulatedVehicle.create_def_header(frame_id=src_id),
                                 point=Point(x=x_pos, y=y_pos, z=0))
            # Don't use the current timestamp, use 0 to use the latest available tf data
            point.header.stamp = rospy.Time(0)
            # Now transform the point using the data
            tf_point = transformer.transformPoint(target_frame=dest_id, ps=point)
            track.box.center_x = tf_point.point.x
            track.box.center_y = tf_point.point.y
        except tf.ExtrapolationException as e:
            # Extrapolation error, print but keep going (possible just because only one frame was received so far)
            print(e)
        except tf.ConnectivityException as e:
            # print("Connectivity Exception during transform")
            print(e)
        except tf.LookupException as e:
            print("Lookup exception")
            pass
    # Now change the frame_id to dest_id, since the tf was already performed
    data.header.frame_id = dest_id

    c2x.append(data)
    steps = 0


def callback_tf(data):
    """
    Uses the tf package for the same operation as callback_tf_manual:
    Append a newly acquired transform message to the list of stored transforms, and plot a c2x message if one was
    recorded already.

    TODO currently is just a test for a single object (since it just takes data.transforms[0]. requires generalization
        can probably achieve this by simply adding the setTransform line in the tracks object? (since its 1 tf per car)
    """
    global transforms, steps, c2x, transformer, time_obj
    c2x_selection = c2x[-1]
    transforms.append(data)
    tf_object = data.transforms[0]
    # Force time == 0 or you will run into issues TODO time==0 force here
    tf_object.header.stamp = rospy.Time(0)
    transformer.setTransform(tf_object)  # set the data as the transform
    if c2x_selection is None:
        return
    lock.acquire()

    tracks = c2x_selection.tracks
    seq = c2x_selection.header.seq

    # transformer: tf.TransformerROS
    for track in tracks:
        time_obj = rospy.Time(0)
        x_pos = track.box.center_x
        y_pos = track.box.center_y
        try:
            head = SimulatedVehicle.create_def_header(frame_id=src_id)
            # Don't use the current timestamp, use 0 to use the latest available tf data
            head.stamp = rospy.Time(0)
            # Try to transform the point
            # Create a point with z=0 for the source frame (out of c2x tracks x/y) coordinate
            point_pos = PointStamped(header=head, point=Point(x=x_pos, y=y_pos, z=0))
            # Now transform the point using the data
            tf_point_pos = transformer.transformPoint(target_frame=dest_id, ps=point_pos)
            # Print/Visualize the point
            # print("("+str(x_pos)+" | "+str(y_pos)+")  -->  ("+str(tf_point.point.x)+" | "+str(tf_point.point.y)+")")
            next_point = (tf_point_pos.point.x, tf_point_pos.point.y, track.object_id, "y")
            visuals.plot_points_tuple([next_point], append=True)
        except tf.ExtrapolationException as e:
            # Extrapolation error, print but keep going (possible just because only one frame was received so far)
            print(e)
    # End of for going over tracks
    steps += 1  # c2x was used in another step, increase the counter
    lock.release()


def callback_tf_static(data):
    """
    Acquires static tf data and adds it to the transformer object.
    """
    global transformer
    for tf_obj in data.transforms:
        # Force time == 0 or you will run into issues TODO time==0 force here
        tf_obj.header.stamp = rospy.Time(0)
        transformer.setTransform(tf_obj)


def callback_tf_append(data):
    """
    Acquires tf data, but does not yet set the transformer. instead this is appended to a list of tf data
    """
    global tf_list
    for tf_obj in data.transforms:
        tf_list.append(tf_obj)


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

    # Currently using _static here because plotting should happen upon receiving lidar data (incl c2x plotting)
    rospy.Subscriber("tf", TFMessage, callback_tf_static)  # Acquire transform messages

    rospy.Subscriber("/FASCarE_ROS_Interface/car2x_objects", TrackedOrientedBoxArray, callback_c2x_tf)
    rospy.Subscriber("tf_static", TFMessage, callback_tf_static)  # Acquire static transform message for "ibeo" frames
    # rospy.Subscriber("tracked_objects/scan", TrackedLaserScan, callback_org_data)

    if len(sys.argv) > 1:
        fname = sys.argv[1]  # Get the filename
        # now start a rosbag play for that filename

        FNULL = open(os.devnull, 'w')
        player_proc = subprocess.Popen(['rosbag', 'play', fname], cwd="data/", stdout=FNULL)

    plt.show()  # DON'T NEED TO SPIN IF YOU HAVE A BLOCKING plt.show

    # Kill the process (if it was started)
    if len(sys.argv) > 1:
        player_proc.terminate()


if __name__ == '__main__':
    steps = 0  # how many steps passed since the last time the c2x message was used
    lock = thr.Lock()
    transforms = []
    # Create a new Visualization object with the axis limits and "blue" as default plotting color
    visuals = TrackVisuals(limit=65, neg_limit=-40, limit_y=50, neg_limit_y=-40, color='b')

    # define a similarity function for t2ta
    # Init the similarity checker that provides the similarity function
    sim_checker = SimilarityChecker(dist_mult=0.1, velo_add=0.4)
    # Select which similarity function should be used
    sim_fct = sim_checker.sim_position
    t2ta_thresh = 8

    # When playing maven-2.bag a passing car gets closer to the c2x track than its actual member - t2ta with history
    # would solve this problem, but the current "simple" similarity checker position-based function can't
    #   maybe the problem can already be solved by incorporating how old the c2x message was, since it doesnt take
    #   into account velocity+age and therefor lags behind its actual position
    # However, when using sim_velocity the system works better
    # Choosing a small threshold (~3) causes the occasional loss of tracking (100 in singleton)
    # Choosing a medium threshold (~8) causes no loss
    # Choosing a bigger threshold will in general cause issues related to other objects getting merged
    # However due to the low amount of "real" vehicles this isn't really an issue (road boundary merging doesnt matter)

    listener()
