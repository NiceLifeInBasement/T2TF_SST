#!/usr/bin/env python
"""
Contains the class definition for the coordinator of the simulation, split from the other classes due its importance and
possible size
"""
import rospy
import numpy as np
from sim_classes import *


class SimulationCoordinator:
    """
    The SimulationCoordinator creates and manages objects and the publishing of data
    """

    vehicles = []  # The list of SimulatedVehicles that is being tracked

    def __init__(self):
        self.vehicles = []

    def sample_init(self, no_objects):
        # Simple init function that creates a few objects
        start_x = 0
        start_y = 0
        offset_x = 8
        offset_y = 0
        vel_x = 2
        vel_y = 1
        for x in range(no_objects):
            next_x =  start_x + offset_x * x
            next_y = start_y + offset_y * x
            next_car = SimulatedVehicle(oid=x, x=next_x, y=next_y, vel_x=vel_x, vel_y=vel_y)
            self.vehicles.append(next_car)
        # Created all vehicles

    def move_all(self, steps=1):
        """
        Performs a basic_move step for all vehicles that are being tracked
        :param steps: How many steps this is supposed to perform on all objects
        """
        for v in self.vehicles:
            v.basic_move(steps=steps)

    def get_true_visual_list(self, color='b'):
        """
        Returns a list of positions that is suitable for display using a TrackVisuals object. The returned lists can
        be passed to the TrackVisuals.plot_points function to display them.
        This uses the TRUE data of the vehicles, that is not noisy.
        :param color: The Color to be used, defaults to blue
        :return: array of (x_pos, y_pos, ids, color) tuples that can be used for plotting
        """
        points = []
        for v in self.vehicles:
            # Create a tuple of the next vehicle as a point
            p = (v.real_center_x, v.real_center_y, v.object_id, color)
            # Add this point to the list of points that already exist
            points.append(p)
        return points
