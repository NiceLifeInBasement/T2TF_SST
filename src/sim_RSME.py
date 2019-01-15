#!/usr/bin/env python
"""
Performs a test run of a simulation
Output: RSME (measurement-groundtruth and fusion-groundtruth)

Performs 2 measurements and CI with those

No Visualization is done

Parameters:
    1: std_dev base used for the measurements
    2: additional std_dev used on top of std_dev (additive) for the second sensor
"""
import rospy
import numpy
import sys
from bob_perception_msgs.msg import *
from simulation.sim_coordinator import *
import time
import math
import copy
from general.t2tf_algorithms import *
import threading as thr



def create_publishers(no_measures=2, qsize=10):

    # Setup a publisher for the ground truth
    pub_truth = rospy.Publisher("t2t_sim/truth", TrackedOrientedBoxArray, queue_size=qsize)
    pub_measure = []
    # Setup a set of publishers, all stored in the pub_measure array
    for i in range(no_measures):
        topic_name = "t2t_sim/measured_"+str(i)
        pub_measure.append(rospy.Publisher(topic_name, TrackedOrientedBoxArray, queue_size=qsize))

    coordinator = SimulationCoordinator()  # Create a new SimulationCoordinator object

    return pub_truth, pub_measure, coordinator


def publish_all(pub_truth, pub_measure, coordinator):
    # Publish data on all publishers: the truth publisher and for every publisher in the array of measurement publishers
    global current_cov_id, stddev, ground_truth_array
    pub_truth.publish(coordinator.get_box_array())
    ground_truth_array.append(coordinator.get_box_array())
    for pub_m in pub_measure:
        global dev_step
        sd = stddev + dev_step*current_cov_id  # Standard deviation for the position of the current measurement
        # Simply using the current_cov_id to alternate between different variances, the scalar value is just to increase
        # the impact of the manipulation (current_cov_id is always 0 or 1 if you have 2 measurements)

        pub_m.publish(coordinator.get_gaussian_box_array(sd_pos=sd, cov_example_id=current_cov_id))
        max_cov_id = 1
        current_cov_id = (current_cov_id + 1) % (max_cov_id + 1)  # Rotate through all possible cov_ids


def setup():
    global ground_truth_array
    ground_truth_array = []
    rospy.init_node("sim_testing", anonymous=True)
    # Setup all global variables
    global stddev, current_cov_id, lock, storage, found_seqs
    stddev = 2  # Default value for the standard deviation of the measured data
    if len(sys.argv) > 1:
        stddev = float(sys.argv[1])
    current_cov_id = 0
    # Default values if nothing was passed via sys.argv
    no_measures = 2
    qsize = 10
    sleep_time = 0.05
    no_steps = 15

    global dev_step
    dev_step = 0
    if len(sys.argv)>2:
        dev_step = float(sys.argv[2])

    lock = thr.Lock()
    storage = []
    found_seqs = []
    subscriber(2)

    # Sample application for repeatedly moving a few cars along a "highway" and publishing this data to the topics
    pub_truth, pub_measure, coordinator = create_publishers(no_measures=no_measures, qsize=qsize)
    coordinator.small_highway_init()
    for s in range(no_steps):
        publish_all(pub_truth, pub_measure, coordinator)
        coordinator.move_all(steps=1)
        # TODO this function is kinda hard to stop via ctrl-c, probably due to the time.sleep function
        time.sleep(sleep_time)


def callback_ci(data):
    """
    Callback that performs covariance intersection information fusion
    association is performed via object ids (which are unchanged from the ground truth and therefore provide perfect
    assoc. results)
    """
    global storage, found_seqs, pub, lock, ground_truth_array
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
                    fused_data.tracks[c].box, omega = fusion(tracked_box.box, other_tracked_box.box)
                    break  # don't need to look further since we found a matching box already
            c += 1  # increment counter
        pub.publish(fused_data)

        """
        # print fusion results:
        for box in fused_data.tracks:
            print("FUSION BOX:")
            print("\tID = "+str(box.object_id))
            print("\tCX = "+str(box.box.center_x))
            print("\tCY = " + str(box.box.center_y))
        print("---")
        for box in ground_truth_array[-1].tracks:
            print("Real BOX:")
            print("\tID = "+str(box.object_id))
            print("\tCX = "+str(box.box.center_x))
            print("\tCY = " + str(box.box.center_y))
        print("#############################")
        """

        # analyse fusion results
        x_diff = []
        y_diff = []
        global rsme_ci
        try:
            rsme_ci
        except NameError:
            rsme_ci = []
        for i in range(len(fused_data.tracks)):
            fusion_box = fused_data.tracks[i]
            real_box = ground_truth_array[-1].tracks[i]
            next_x_diff = fusion_box.box.center_x - real_box.box.center_x
            x_diff.append(next_x_diff * next_x_diff)  # square error for x
            next_y_diff = fusion_box.box.center_y - real_box.box.center_y
            y_diff.append(next_y_diff * next_y_diff)  # square error for y
        rsme_x_ci = math.sqrt(sum(x_diff)/len(x_diff))
        rsme_y_ci = math.sqrt(sum(y_diff) / len(y_diff))
        rsme_ci.append(math.sqrt(rsme_x_ci**2 + rsme_y_ci**2))  # pythagoras for xy distance

        # data contains the necessary information:
        x_diff = []
        y_diff = []
        global rsme_data
        try:
            rsme_data
        except NameError:
            rsme_data = []
        for i in range(len(data.tracks)):
            fusion_box = data.tracks[i]
            real_box = ground_truth_array[-1].tracks[i]
            next_x_diff = fusion_box.box.center_x - real_box.box.center_x
            x_diff.append(next_x_diff * next_x_diff)  # square error for x
            next_y_diff = fusion_box.box.center_y - real_box.box.center_y
            y_diff.append(next_y_diff * next_y_diff)  # square error for y
        rsme_x_data = math.sqrt(sum(x_diff) / len(x_diff))
        rsme_y_data = math.sqrt(sum(y_diff) / len(y_diff))
        rsme_data.append(math.sqrt(rsme_x_data ** 2 + rsme_y_data ** 2))  # pythagoras for xy distance

        # other_data contains the necessary information:
        x_diff = []
        y_diff = []
        global rsme_other_data
        try:
            rsme_other_data
        except NameError:
            rsme_other_data = []
        for i in range(len(other_data)):
            fusion_box = other_data[i]
            real_box = ground_truth_array[-1].tracks[i]
            next_x_diff = fusion_box.box.center_x - real_box.box.center_x
            x_diff.append(next_x_diff * next_x_diff)  # square error for x
            next_y_diff = fusion_box.box.center_y - real_box.box.center_y
            y_diff.append(next_y_diff * next_y_diff)  # square error for y
        rsme_x_other_data = math.sqrt(sum(x_diff) / len(x_diff))
        rsme_y_other_data = math.sqrt(sum(y_diff) / len(y_diff))
        rsme_other_data.append(math.sqrt(rsme_x_other_data ** 2 + rsme_y_other_data ** 2)) # pythagoras for xy distance)

        # ---
        global em_count
        try:
            em_count += 1
        except NameError:
            em_count = 0
        print("ERROR MEASUREMENT #"+str(em_count)+"[omega="+str(omega)+"]:")
        print("\tCI: " + str(sum(rsme_ci)/len(rsme_ci)))
        print("\tM1: " + str(sum(rsme_data)/len(rsme_data)))
        print("\tM2: " + str(sum(rsme_other_data)/len(rsme_other_data)))
        # m,c = min(averages of the measuements), average for ci
        m = min(sum(rsme_data)/len(rsme_data), sum(rsme_other_data)/len(rsme_other_data))
        c = sum(rsme_ci)/len(rsme_ci)
        # use m+c to calculate the improvement of the CI over the better measurement
        # if c=2, and m=3, then the improvement should be 33.3% == (m-c)/m*100
        print("\tI%: " + str((m-c)/m*100))

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
    :return:    Fused Track, in OrientedBox Format
                omega used
    """
    estimate = copy.deepcopy(track_i)  # Create an OrientedBox based on track i
    # PRESETS:
    t2tf = dual_cov_intersection
    # w_calc = dual_improved_omega  # Alt.: dual_fast_omega
    w_calc = dual_fast_omega
    # ---
    P_i = to_cov_mat(track_i)  # Covariance matrix of track i
    P_j = to_cov_mat(track_j)  # Covariance matrix of track j

    x_i = [track_i.center_x, track_i.center_y, track_i.velocity_x, track_i.velocity_y]  # Tracking estimate of track i
    x_i = np.array(x_i)
    x_j = [track_j.center_x, track_j.center_y, track_j.velocity_x, track_j.velocity_y]  # Tracking estimate of track j
    x_j = np.array(x_j)
    x, P, w = t2tf(P_i, P_j, x_i, x_j, omega_fct=w_calc)
    # print("omega: "+str(w))  # Print out the used omega for DEBUG purposes
    # Update the estimate based on x
    estimate.center_x = x[0]
    estimate.center_y = x[1]
    estimate.velocity_x = x[2]
    estimate.velocity_y = x[3]

    return estimate, w


def subscriber(no_measurements):
    # no_measurements should be 2 for now
    # since I use only one array to store data (so only the incoming track and one stored track are compared)
    global pub
    for i in range(no_measurements):
        tname = "/t2t_sim/measured_"+str(i)
        rospy.Subscriber(tname, TrackedOrientedBoxArray, callback_ci)
    pub = rospy.Publisher("t2t_sim/fused_CI", TrackedOrientedBoxArray, queue_size=10)


if __name__ == '__main__':
    print("Running main of RSME simulation testing")
    setup()
