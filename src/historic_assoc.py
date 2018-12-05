#!/usr/bin/env python
from __future__ import print_function
"""
Similar to general_plot.py, but uses historic T2TA instead.

This can play a bag file that includes multiple data sources and fuse them using T2TA approach that is incorporating
history into association.
The bag data is plotted as usual.
Association results are printed to the commandline, but can of course also be published to a new topic or handed over
to a fusion centre that then performs T2TF with the resulting data
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

# --- Definiton of global variables
# [...]

# --- callback functions
def callback_tracking(data):
    """
    Plot data from the original laser scans and from the c2x scan to the visuals object and additionally perform
    T2TA on this data, printing the results to the command line
    """
    # insert the basic code like in general_plot.py here to display everything etc
    # additionally, add the t2t_history code that updates the t2th object and calls the t2ta_historic code
    global visuals, lock, transforms, steps, c2x, inc_c2x, history, t2ta_thresh, state_space, use_identity
    lock.acquire()
    c2x_selection = closest_match(c2x, data.header.stamp)
    if c2x_selection is not None:
        # Remove the selected c2x entry from the list of all c2x entries so that it doesn't get used twice
        c2x.remove(c2x_selection)
    visuals.plot_box_array(data.boxes, append=False)
    # Append the current information to the history object
    history.add("lidar_0", data.boxes)

    if c2x_selection is not None:
        # Also include c2x data in the plot

        # ---
        # The following is the c2x data transformation, where the c2x pos/vel etc are transformed into the same coord.
        # frame as the tracking data that was received in this time step.
        # For this, all tracks in the c2x data are transformed using the transformer object.
        # Afterwards, T2TA is attempted on the resulting data. Results of this are printed to console, but currently
        # not passed on in any way. A future implementation should probably include a publisher, that publishes the
        # resulting data to a new topic, that a t2tf client can subscribe to.
        # ---

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
                next_point = (x_pos, y_pos, track.object_id, "y")
                visuals.plot_points_tuple([next_point], append=True)
            except tf.ExtrapolationException as e:
                # Extrapolation error, print but keep going (possible just because only one frame was received so far)
                print(e)
            except tf.ConnectivityException as e:
                # print("Connectivity Exception during transform")
                print(e)
        # End of for going over tracks
        steps += 1  # c2x was used in another step, increase the counter

        # Now to the T2TA with history:
        try:
            # generate object ids and sensor names
            lidar_ids = []
            c2x_ids = []
            for obj in data.boxes:
                lidar_ids.append(obj.object_id)
            for obj in c2x_selection.tracks:
                c2x_ids.append(obj.object_id)
            obj_ids = [lidar_ids, c2x_ids]

            sensor_names = ["lidar_0", "c2x_0"]

            # Data acquisition from the history object should be based on the most recently received objects stamp
            timing = data.boxes[0].box.header.stamp
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
                    pass
        except ValueError as e:
            print("ValueError during association")
            # print(e)
        except IndexError as e:
            print("IndexError during association, likely because not enough data was received yet.")

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

def callback_c2x_tf(data):
    """
    Stores the last c2x message, but transforms it beforehand
    """
    global history, steps, constant_velo, c2x_offset_x, c2x_offset_y
    tracks = data.tracks
    head = SimulatedVehicle.create_def_header(frame_id=src_id)
    # transformer: tf.TransformerROS
    for track in tracks:
        time_obj = rospy.Time(0)
        x_pos = track.box.center_x
        y_pos = track.box.center_y
        x_vel = track.box.velocity_x
        y_vel = track.box.velocity_y
        try:
            # Try to transform the point
            # Create a point with z=0 for the source frame (out of c2x tracks x/y) coordinate
            point = PointStamped(header=head,
                                 point=Point(x=x_pos, y=y_pos, z=0))
            # Don't use the current timestamp, use 0 to use the latest available tf data
            point.header.stamp = rospy.Time(0)
            # Now transform the point using the data
            tf_point = transformer.transformPoint(target_frame=dest_id, ps=point)

            # Acquire the transformation matrix for this
            tf_mat = tf_c.toMatrix(tf_c.fromTf(transformer.lookupTransform(target_frame=dest_id, source_frame=src_id, time=rospy.Time(0))))
            # print(tf_mat)

            # tf_vel stores the transformed velocity
            tf_vel = np.dot(tf_mat[0:2, 0:2], [x_vel, y_vel])
            # Update the track with the transformed data
            track.box.center_x = tf_point.point.x + c2x_offset_x
            track.box.center_y = tf_point.point.y + c2x_offset_y
            track.box.velocity_x = tf_vel[0]
            track.box.velocity_y = tf_vel[1]
            # DEBUG
            # print(str(track.box.center_x)+"\t"+str(track.box.center_y))
            # print("steps: "+str(steps)+"\tvelx: "+str(point_vel.point.x)+" vely: "+str(point_vel.point.y))
        except tf.ExtrapolationException as e:
            # Extrapolation error, print but keep going (possible just because only one frame was received so far)
            print(e)
        except tf.ConnectivityException as e:
            # print("Connectivity Exception during transform")
            print(e)
        except tf.LookupException as e:
            print(e)
            pass
    # Now change the frame_id to dest_id, since the tf was already performed
    data.header.frame_id = dest_id

    c2x.append(data)
    history.add("c2x_0", data.tracks)  # Add the data to the history object under the name c2x_0

    # The following block adds several fake measurements to the c2x tracking that are time delayed to the original
    # measurement from the current timestep
    # They also have changed x/y data based on velocity, so that the track doesn't need to be reused later on with the
    # outdated coordinates and instead can work with "updated" coordinates for time steps that are not included in the
    # real c2x messages.
    add_points = 4  # how many "fake" points should be added between this and the next measurement
    # TODO consider changing the timing/number of fakes depending on the time diff between this data and the last data
    if len(data.tracks) > 0:  # only do the following if the track contained something
        for i in range(add_points):
            # add 4 more measurements, each based on the data, but with manipulated time and position based on velocity
            fake_data = copy.deepcopy(data)  # Create a copy of the data
            # now change the timestamp of this new data
            # c2x data arrives every 0.2 seconds, so to fit an additional 4 measurements in there
            time_shift = rospy.Duration(0, 40000000)  # increase by 1/20 of a sec
            fake_data.header.stamp = fake_data.header.stamp + time_shift
            for track in fake_data.tracks:
                track.box.header.stamp = track.box.header.stamp + time_shift
                track.box.center_x += constant_velo * track.box.velocity_x * (i+1)
                track.box.center_y += constant_velo * track.box.velocity_y * (i+1)
            c2x.append(fake_data)
            history.add("c2x_0", fake_data.tracks)

    steps = 0


# --- listener and global stuff


def listener(args):
    """
    Prepare the subscribers and setup the plot etc
    :param args: The programs arguments, usually sys.argv unless you called setup from a different functionx
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

    if len(args) > 1:
        fname = args[1]  # Get the filename
        # now start a rosbag play for that filename
        FNULL = open(os.devnull, 'w')  # redirect rosbag play output to devnull to suppress it

        play_rate = 0.15  # set the number to whatever you want your play-rate to be
        rate = '-r' + str(play_rate)
        # using '-r 1' is the usual playback speed - this works, but since the code lags behind (cant process everything
        # in realtime), you will then get results after the bag finished playing (cached results)
        # using '-r 0.25' is still too fast for maven-1.bag
        # using '-r 0.2' works (bag finishes and no more associations are made on buffered data afterwards)

        start_time = 25  # time at which the bag should start playing
        time = '-s ' + str(start_time)
        if start_time > 0:
            pkl_filename = "./src/T2TF_SST/data/"  # folder
            pkl_filename += "tf_static_dump.pkl"  # filename
            with open(pkl_filename, 'rb') as pklinput:
                tf_static_data = pickle.load(pklinput)
                callback_tf_static(tf_static_data)

        player_proc = subprocess.Popen(['rosbag', 'play', rate, time, fname], cwd="data/", stdout=FNULL)

    plt.show()  # DON'T NEED TO SPIN IF YOU HAVE A BLOCKING plt.show

    # Kill the process (if it was started)
    if len(args) > 1:
        player_proc.terminate()


def setup(args=None):
    """
    Setup everything and call the listener function
    :param args: If None, sys.argv will be used as a parameter for the listener, else this should simulate cmdline
    parameters, so it should be ['path/to/file.py', 'bag_name'] where bagname is the name of the bag in the data folder.
    """
    global steps, src_id, dest_id, dest_id_vel, c2x, lock, history, hist_size, state_space, use_identity, transforms, t2ta_thresh, constant_velo, visuals, c2x_offset_x, c2x_offset_y
    steps = 0  # how many steps passed since the last time the c2x message was used
    src_id = "odom"  # transformations are performed FROM this frame
    dest_id = "ibeo_front_center"  # transformations are performed TO this frame
    dest_id_vel = "ibeo_front_center"  # frame to use for velocity transformation (equiv. to dest_id but for velo)
    c2x = []  # Array that is used to store all incoming c2x messages
    lock = thr.Lock()
    history = TrackingHistory()
    # hist_size = rospy.Duration(0) => history will be only 1 track (i.e. no history)
    # hist size Duration(4) causes significant lag already!
    hist_size = rospy.Duration(0, 500000000)  # .5 secs
    # hist_size = rospy.Duration(0)

    state_space = (True, False, False, False)  # usual state space: (TFFT), only pos: (TFFF)
    # The threshold NEEDS TO BE ADJUSTED if you use something other than TFFF!

    use_identity = True

    transforms = []
    # Create a new Visualization object with the axis limits and "blue" as default plotting color
    visuals = TrackVisuals(limit=65, neg_limit=-40, limit_y=50, neg_limit_y=-40, color='b')

    # define a similarity function for t2ta
    # Init the similarity checker that provides the similarity function
    # 20 works safe, but leads to some wrong associations between c2x data and road boundaries in very few scenarios
    # reduced it to 13 for now, but without doing extensive testing
    t2ta_thresh = 13  # Threshold for using only position data
    # t2ta_thresh = 25  # Threshold for using position + velocity data

    # value that is multiplied with velocity to acquire the position of the c2x objects in "untracked" time steps
    # i.e. between messages (since the frequency of the messages is lacking)
    # factor that needs to be used for velo when looking at change between 2 consecutive steps
    constant_velo = 0.05  # 0.05 performs best in my tests using maven-1.bag (if using c2x_offset_x=4)
    # The new transformation of velocity using the extracted matrix works, so you can now use a "normal" factor.
    float("inf")
    # Constant offsets for all c2x data
    # TODO possibly "overfitting", but works incredibly well on both bag files
    # I assume the reason that this works/is necessary could be because the trackers have different "centers of tracking
    # boxes, and therefore this offset is necessary to "skip" the distance between the points
    c2x_offset_x = 4
    c2x_offset_y = 0

    # Check which arguments should be used (parameter if possible, else sys.argv)
    if args is None:
        listener(sys.argv)
    else:
        listener(args)


if __name__ == '__main__':
    # Simply call the setup function, don't pass args so that sys.argv is used instead
    setup()

