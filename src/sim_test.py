#!/usr/bin/env python
"""
Testing class for the simulation
"""
import rospy
import roslib; roslib.load_manifest('T2TF_SST')
import numpy as np
from simulation.sim_coordinator import *
from tracking_consistency.tracking_visuals import *
import time

if __name__ == '__main__':
    coordinator = SimulationCoordinator()
    coordinator.sample_init(no_objects=10)
    vis = TrackVisuals(limit=100, neg_limit=-10)
    for s in range(25):
        coordinator.move_all()
        point_list = coordinator.get_true_visual_list(color='b')
        vis.plot_points_tuple(points=point_list, append=False)

        # Change the velocity of a random object
        pick = np.random.randint(len(coordinator.vehicles))
        speed = np.random.uniform(low=0.5, high=1.5)
        speed_y = np.random.uniform(low=0.5, high=1.5)
        coordinator.vehicles[pick].basic_accelerate(factor=speed, factor_y=speed_y)

        time.sleep(0.2)  # pause for a second so that it doesnt rush through
