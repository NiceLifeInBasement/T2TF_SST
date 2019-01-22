#!/usr/bin/env python
"""
Performs track to track association on simulated data.

Currently used to test the t2ta algorithm.

Assumes synchronization has been done for all tracks, sequence ids are used to match incoming messages to each other.
If sequence numbers are offset this needs to be cleaned before the data is passed to this program.
However, since this is using simulated data, this should not be an issue and not be a big focus until synchronizing real
data become the main problem.

Some notes on the current stage of testing (13-Nov-18 / 15:45)
    Using a very low threshold reduces the number of clusters that contain wrong ids since then all clusters will be
    singletons (i.e. splitting of tracks has happened for every measurement!)

    Using a very high threshold such as 50 prevents any singleton clusters from spawning. A threshold of 10 is not
    quite enough for this.

    creation of singleton clusters is a (pretty safe) sign for object splitting, i.e. after association more objects
    than actually exist are in the tracking

    using velo_add=0.5 for the similarity checker works fine-ish
"""

from bob_perception_msgs.msg import *
import threading as thr
import general.t2ta_algorithms as t2ta
from similarity import *


def callback_association(data):
    """
    Performs an association for the incoming data if data for the same sequence id has already been saved previously,
    or saves data for future use if no data is stored for this seq. id
    :param data: TrackedOrientedBoxArray of a sensor track
    """
    global storage, found_seqs, sim_fct
    lock.acquire()  # Using a lock to prevent issues in the array management (all length checks etc would cause issues)

    # Data is assumed to be synchronized by sequence id, so use this to access data from the same time step
    seq = data.header.seq
    seq -= 1  # -1 because its stored in an array which is zero-based while the ids are not
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
        # Sequence was not yet found in the data, add it to the storage of data at the correct position (=its seq. id)
        # First, check if this is actually new data (if not, print a warning message)
        if not len(storage) == seq:
            # Length of the current data storage and sequence id dont match, so the data is from an outdated time step
            # Print a warning to the console.
            print("Issue at seq "+str(seq)+"\tdoesnt match storage with length"+str(len(storage)))
        storage.append(data)
        found_seqs.append(seq)

    lock.release()  # finished, release the lock


def subscriber(no_measurements=2):
    # no_measurements should be 2 for now (since only one array is used to store data, so only data from that one array
    # and data that came in in that time step can be used)
    # since this uses only one array to store data (so only the incoming track and one stored track are compared)
    global pub
    rospy.init_node("T2TA_Test", anonymous=True)
    for i in range(no_measurements):
        tname = "/t2t_sim/measured_"+str(i)
        rospy.Subscriber(tname, TrackedOrientedBoxArray, callback_association)

    rospy.spin()


if __name__ == '__main__':
    # Init the similarity checker that provides the similarity function
    sim_checker = SimilarityChecker(dist_mult=0.1, velo_add=0.4)

    # Select which similarity function should be used
    sim_fct = sim_checker.sim_position
    # sim_fct = sim_checker.sim_velocity
    # Create all the necessary global variables
    lock = thr.Lock()
    storage = []
    found_seqs = []
    subscriber(no_measurements=2)
