#!/usr/bin/env python
"""
Publishes data to the relevant topics:
t2t_sim/truth:
    True position data in TrackedOrientedBoxArray format
t2t_sim/measured_<X>:
    Data that was "measured" i.e. has noise/measurement errors.
    <X> is a number in [0, number_of_trackers)
    number_of_trackers means how many measurement should be acquired in parallel


USAGE:
0. Parameters:
        1. float: std deviation for the measurements
1. use create_publishers to create all publishers for the topic and the coordinator object
2. init the coordinator object in whatever way you want
3. whenever you want to publish the measurements of the cars to all channels, call publish_all
4. in between these publishing steps you should use coordinator.move_all etc to change the actual data
5. a different client can subscribe to these topics to receive all data about the measured data
"""
from simulation.sim_coordinator import *
from tracking_visuals import *
import time
import bob_perception_msgs.msg as bobmsg


def create_publishers(no_measures=2, qsize=10):
    rospy.init_node("t2t_simulation_publisher", anonymous=True)

    # Setup a publisher for the ground truth
    pub_truth = rospy.Publisher("t2t_sim/truth", bobmsg.TrackedOrientedBoxArray, queue_size=qsize)
    pub_measure = []
    # Setup a set of publishers, all stored in the pub_measure array
    for i in range(no_measures):
        topic_name = "t2t_sim/measured_"+str(i)
        pub_measure.append(rospy.Publisher(topic_name, bobmsg.TrackedOrientedBoxArray, queue_size=qsize))

    coordinator = SimulationCoordinator()  # Create a new SimulationCoordinator object

    return pub_truth, pub_measure, coordinator


def publish_all(pub_truth, pub_measure, coordinator):
    # Publish data on all publishers: the truth publisher and for every publisher in the array of measurement publishers
    global current_cov_id, stddev
    pub_truth.publish(coordinator.get_box_array())
    for pub_m in pub_measure:
        sd = stddev + 1*current_cov_id  # Standard deviation for the position of the current measurement
        # Simply using the current_cov_id to alternate between different variances, the scalar value is just to increase
        # the impact of the manipulation (current_cov_id is always 0 or 1 if you have 2 measurements)

        pub_m.publish(coordinator.get_gaussian_box_array(sd_pos=sd, cov_example_id=current_cov_id))
        max_cov_id = 1
        current_cov_id = (current_cov_id + 1) % (max_cov_id + 1)  # Rotate through all possible cov_ids


if __name__ == '__main__':
    # Setup all global variables
    stddev = 2  # Default value for the standard deviation of the measured data
    if len(sys.argv) > 1:
        stddev = float(sys.argv[1])
    current_cov_id = 0
    # Default values if nothing was passed via sys.argv
    no_measures = 2
    qsize = 10
    sleep_time = 1
    no_steps = 15

    # Sample application for repeatedly moving a few cars along a "highway" and publishing this data to the topics
    pub_truth, pub_measure, coordinator = create_publishers(no_measures=no_measures, qsize=qsize)
    coordinator.small_highway_init()
    for s in range(no_steps):
        publish_all(pub_truth, pub_measure, coordinator)
        coordinator.move_all(steps=1)
        time.sleep(sleep_time)  # makes the function somewhat hard to stop, but is necessary for slowed exec.
