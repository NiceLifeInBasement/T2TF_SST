#!/usr/bin/env python
"""
Subscribes to two measurements of simulated data, converts the data into numpy arrays and estimates the position
of the vehicle based on a CI algorithm
"""

import rospy
import numpy as np
from bob_perception_msgs.msg import *
from std_msgs.msg import Float64MultiArray
import threading as thr
import copy
from general.t2tf_algorithms import *
import math


def callback_ci(data):
    global storage, found_seqs, pub
    lock.acquire()

    seq = data.header.seq
    seq -= 1
    if seq in found_seqs:
        # found this sequence in the data, assuming that its position matches its sequence number
        fused_data = copy.deepcopy(data)
        other_data = storage[seq].tracks
        new_data = data.tracks
        c = 0  # counter for which box is currently being edited
        for tracked_box in new_data:
            # first, find a matching box in the other_data:
            oid = tracked_box.object_id
            for other_tracked_box in other_data:
                if other_tracked_box.object_id == oid:
                    # found a matching one
                    # Perform ci on the data
                    fused_data.tracks[c].box = fusion(tracked_box.box, other_tracked_box.box)
                    break  # dont need to look further since we found a matching box already
            c += 1  # increment counter
        pub.publish(fused_data)
    else:
        if not len(storage) == seq:
            print("Issue at seq "+str(seq)+"\tdoesnt match storage with length"+str(len(storage)))
        storage.append(data)
        found_seqs.append(seq)
    lock.release()


def to_cov_mat(box):
    """
    Takes an OrientedBox as input and returns a numpy array of the covariance matrix
    :param box: The OrientedBox from which the covariance matrix should be extracted
    :return: A np array that includes the covariance matrix from the box, not including nans
    """
    cov_list = box.covariance.data
    cov_list = [x for x in cov_list if not math.isnan(x)]  # Remove all nans
    cov_list = np.array(cov_list)  # Turn into numpy array
    dim = 0  # the resulting array will be of shape (dim x dim)

    # ASSUMING THAT THE ORDER GIVEN IN THE DOCUMENTATION OF ORIENTED BOX IS CORRECT
    # usually you will only have box.covariance_center=True and box.covariance_velocity=True
    # And the resulting matrix will be a 4x4 matrix
    if box.covariance_center:
        dim += 2
    if box.covariance_angle:
        dim += 1
    if box.covariance_length_width:
        dim += 2
    if box.covariance_velocity:
        dim += 2
    cov_matrix = np.reshape(cov_list, (dim, dim))
    return cov_matrix


def fusion(track_i, track_j):
    """
    Takes in two tracks and fuses them according to the CI algorithm
    :param track_i: First track, in OrientedBox Format
    :param track_j: Second track, in OrientedBox Format
    :return: Fused Track, in OrientedBox Format
    """
    estimate = copy.deepcopy(track_i)  # Create an OrientedBox based on track i
    # PRESETS:
    t2tf = dual_cov_intersection
    w_calc = dual_fast_omega  # Alt.: dual_improved_omega

    # ---
    P_i = to_cov_mat(track_i)  # Covariance matrix of track i
    P_j = to_cov_mat(track_j)  # Covariance matrix of track j

    x_i = [track_i.center_x, track_i.center_y, track_i.velocity_x, track_i.velocity_y]  # Tracking estimate of track i
    x_i = np.array(x_i)
    x_j = [track_j.center_x, track_j.center_y, track_j.velocity_x, track_j.velocity_y]  # Tracking estimate of track j
    x_j = np.array(x_j)
    x, P, w = t2tf(P_i, P_j, x_i, x_j, omega_fct=w_calc)

    # Update the estimate based on x
    estimate.center_x = x[0]
    estimate.center_y = x[1]
    estimate.velocity_x = x[2]
    estimate.velocity_y = x[3]

    return estimate


def subscriber(no_measurements):
    # no_measurements should be 2 for now
    # since I use only one array to store data (so only the incoming track and one stored track are compared)
    global pub
    rospy.init_node("CI_fusion", anonymous=True)
    for i in range(no_measurements):
        tname = "/t2t_sim/measured_"+str(i)
        rospy.Subscriber(tname, TrackedOrientedBoxArray, callback_ci)
    pub = rospy.Publisher("t2t_sim/fused_CI", TrackedOrientedBoxArray, queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    lock = thr.Lock()
    storage = []
    found_seqs = []
    subscriber(2)