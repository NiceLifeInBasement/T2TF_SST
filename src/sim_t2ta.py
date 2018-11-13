#!/usr/bin/env python
"""
Performs track to track association on simulated data.

Currently used to test the t2ta algorithm.

Some notes on the current stage of testing (13-Nov-18 / 15:45)
    TODO find out why there is 24 tracks when 8 cars are tracked by 2 sensors. Possibly t2ta error!!!
    Using a low threshold reduces the number of clusters that contain wrong ids since then all clusters will be
    singletons

    Using a very high threshold such as 50 prevents any singleton clusters from spawning. A threshold of 10 is not
    quite enough for this.

    creation of singleton clusters is a (pretty safe) sign for object splitting, i.e. after association more objects
    than actually exist are in the tracking

    using velo_add=0.5 for the similarity checker works fine-ish
"""

import rospy
import numpy as np
from bob_perception_msgs.msg import *
from std_msgs.msg import Float64MultiArray
import threading as thr
import copy
from general.t2tf_algorithms import *
import math
import general.t2ta_algorithms as t2ta
from tracking_consistency.similarity import *


def callback_association(data):
    global storage, found_seqs, sim_fct
    lock.acquire()

    seq = data.header.seq
    seq -= 1
    if seq in found_seqs:
        # found this sequence in the data, assuming that its position matches its sequence number
        # perform an association step
        other_data = storage[seq].tracks
        new_data = data.tracks
        assoc = t2ta.t2ta_collected([other_data, new_data], threshold=20, distance=sim_fct)

        # analyse the results
        ids = []  # this list will hold lists where each entry is an object id in a cluster
        no_wrong_clusters = 0  # how many clusters contained an error
        no_singletons = 0  # how many singleton clusters are in the list
        for a in assoc:
            temp = []
            for box in a:
                temp.append(box.object_id)
            ids.append(temp)
            # convert the list for this cluster into a set and check its length to find number unique entries
            if len(set(temp)) > 1:
                no_wrong_clusters += 1
            if len(temp) == 1:
                no_singletons += 1
        print(str(no_wrong_clusters)+" mismatched Clusters.\t"+str(no_singletons)+" Singleton Clusters.\t"+str(len(ids))
              + " associated objects. Full List:")
        print(ids)
        print(" ")

    else:
        if not len(storage) == seq:
            print("Issue at seq "+str(seq)+"\tdoesnt match storage with length"+str(len(storage)))
        storage.append(data)
        found_seqs.append(seq)

    lock.release()


def subscriber(no_measurements):
    # no_measurements should be 2 for now
    # since I use only one array to store data (so only the incoming track and one stored track are compared)
    global pub
    rospy.init_node("T2TA_Test", anonymous=True)
    for i in range(no_measurements):
        tname = "/t2t_sim/measured_"+str(i)
        rospy.Subscriber(tname, TrackedOrientedBoxArray, callback_association)

    rospy.spin()


if __name__ == '__main__':
    sim_checker = SimilarityChecker(dist_mult=0.1, velo_add=0.5)
    sim_fct = sim_checker.sim_position
    lock = thr.Lock()
    storage = []
    found_seqs = []
    subscriber(2)
