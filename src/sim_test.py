#!/usr/bin/env python
"""
Testing class for the simulation
"""

# Appears to be not necessary, would be for being able to import my own modules
# import roslib; roslib.load_manifest('T2TF_SST')

from simulation.sim_coordinator import *
from tracking_visuals import *
import time


def random_sim():
    # Inits a bunch of points and moves them across the board
    # In every time step, one of the cars is accelerated (random pick, random speed)
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

        time.sleep(0.2)  # pause for a bit so that it doesnt rush through


def multi_lane_sim():
    # Deterministic Simulation of a cars on a highway (both directions, two lanes)

    sd_pos = 1
    sd_vel = 0.05

    coord = SimulationCoordinator()
    vis = TrackVisuals(limit=50, neg_limit=-50)

    coord.small_highway_init()

    for s in range(50):
        coord.move_all()
        point_list = coord.get_true_visual_list(color='b')
        noisy_point_list = coord.get_gaussian_visual_list(stddev_pos=sd_pos, stddev_vel=sd_vel)
        point_list.extend(noisy_point_list)
        vis.plot_points_tuple(points=point_list, append=False)
        time.sleep(0.2)


if __name__ == '__main__':
    # Just pick which simulation you want to start here
    multi_lane_sim()
