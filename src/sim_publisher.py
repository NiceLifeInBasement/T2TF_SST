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
use create_publishers to create all publishers for the topic and the coordinator object
init the coordinator object in whatever way you want
whenever you want to publish the measurements of the cars to all channels, call publish_all
in between these publishing steps you should use coordinator.move_all etc to change the actual data
A different client can subscribe to these topics to receive all data about the measured data
"""
import rospy
import numpy as np
from simulation.sim_coordinator import *
from tracking_consistency.tracking_visuals import *
import time
import bob_perception_msgs.msg as bobmsg


def create_publishers(no_measures=2, qsize=10):
    rospy.init_node("t2t_simulation_publisher", anonymous=True)

    pub_truth = rospy.Publisher("t2t_sim/truth", bobmsg.TrackedOrientedBoxArray, queue_size=qsize)
    pub_measure = []
    for i in range(no_measures):
        topic_name = "t2t_sim/measured_"+str(i)
        pub_measure.append(rospy.Publisher(topic_name, bobmsg.TrackedOrientedBoxArray, queue_size=qsize))

    coordinator = SimulationCoordinator()  # Create a new SimulationCoordinator object

    # TODO find a way to pass a function for the following, instead of hard-coding the function here

    return pub_truth, pub_measure, coordinator


def publish_all(pub_truth, pub_measure, coordinator):
    pub_truth.publish(coordinator.get_box_array())
    for pub_m in pub_measure:
        pub_m.publish(coordinator.get_gaussian_box_array())


if __name__ == '__main__':
    # Default values if nothing was passed via sys.argv
    no_measures = 2
    qsize = 10

    # Sample application for repeatedly moving a few cars along a "highway" and publishing this data to the topics
    pub_truth, pub_measure, coordinator = create_publishers(no_measures=no_measures, qsize=qsize)
    coordinator.small_highway_init()
    for s in range(50):
        publish_all(pub_truth, pub_measure, coordinator)
        coordinator.move_all(steps=1)
        time.sleep(0.2)
