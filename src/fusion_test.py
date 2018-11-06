#!/usr/bin/env python
"""
subscribes to two measurements of simulated data, fuses them together and publishes the result

Currently just averaging between two datapoints
"""
import rospy
import numpy as np
from bob_perception_msgs.msg import *
import threading as thr
import copy


def callback(data):
    # currently using two arrays (data+sequences), can just go over the storage array and check ids though
    # which would be a lot less prone to errors
    # its possible that the updating doesnt work live right now, sometimes it looks like outdated data is being
    # displayed (so data from an older sequence gets displayed/used somehow)

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
                    # recalc x + y by averaging
                    new_x = (tracked_box.box.center_x + other_tracked_box.box.center_x) / 2
                    new_y = (tracked_box.box.center_y + other_tracked_box.box.center_y) / 2
                    fused_data.tracks[c].box.center_x = new_x
                    fused_data.tracks[c].box.center_y = new_y
                    break  # dont need to look further since we found a matching box already
            c += 1  # increment counter
        pub.publish(fused_data)
    else:
        if not len(storage)==seq:
            print("Issue at seq "+str(seq)+"\tdoesnt match storage with length"+str(len(storage)))
        storage.append(data)
        found_seqs.append(seq)
    lock.release()


def subscriber(no_measurements):
    # no_measurements should be 2 for now
    # since I use only one array to store data (so only the incoming track and one stored track are compared)
    global pub
    rospy.init_node("average_fusion", anonymous=True)
    for i in range(no_measurements):
        tname = "/t2t_sim/measured_"+str(i)
        rospy.Subscriber(tname, TrackedOrientedBoxArray, callback)
    pub = rospy.Publisher("t2t_sim/fused", TrackedOrientedBoxArray, queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    lock = thr.Lock()
    storage = []
    found_seqs = []
    subscriber(2)
