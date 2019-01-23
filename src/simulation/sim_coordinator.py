#!/usr/bin/env python
"""
Contains the class definition for the coordinator of the simulation, split from the other classes due its importance and
possible size
TODO longer description of the file
"""
import rospy
import numpy as np
from sim_classes import *
import bob_perception_msgs.msg as bobmsg


class SimulationCoordinator:
    """
    The SimulationCoordinator creates and manages objects and the publishing of data
    """

    vehicles = []  # The list of SimulatedVehicles that is being tracked

    def __init__(self):
        self.vehicles = []

    def sample_init(self, no_objects):
        """
        Simple init function that creates a few objects along the x and y axis. The vehicles will all be in different
        positions, but with the same velocity
        :param no_objects: How many objects should be added
        """
        start_x = -20
        start_y = -20
        offset_x = 8
        offset_y = 0
        vel_x = 2
        vel_y = 1
        for x in range(no_objects):
            next_x = start_x + offset_x * x
            next_y = start_y + offset_y * x
            next_car = SimulatedVehicle(oid=x, x=next_x, y=next_y, vel_x=vel_x, vel_y=vel_y)
            self.vehicles.append(next_car)
        # Created all vehicles

    def add_vehicle(self, oid, x=0.0, y=0.0, angle=0.0, length=0.0, width=0.0, vel_x=0.0, vel_y=0.0):
        """
        Adds a new vehicle to the list of vehicles that is being simulated.
        :param oid: Object ID of the new vehicle
        :param x: x position of the new vehicle
        :param y: y position of the new vehicle
        :param angle: angle of the new vehicle
        :param length: length of the tracking box of the new vehicle
        :param width: width of the tracking box of the new vehicle
        :param vel_x: velocity in x direction of the new vehicle
        :param vel_y: velocity in y direction of the new vehicle
        :return:
        """
        v = SimulatedVehicle(oid, x, y, angle, length, width, vel_x, vel_y)
        self.vehicles.append(v)

    def small_highway_init(self):
        """
        Very simple init function that creates a few cars that move along a "highway" in both directions (up/down) and
        in two lanes. 3 cars per side will be going at a slow pace (right lane) and one will pass them in the left lane.
        """
        # Simple init function that creates a few cars that drive along a small highway
        self.add_vehicle(0, 10, -50, 0, 0, 0, 0, 2)
        self.add_vehicle(1, 10, -44, 0, 0, 0, 0, 2)
        self.add_vehicle(2, 10, -34, 0, 0, 0, 0, 2)
        self.add_vehicle(3, 5, -55, 0, 0, 0, 0, 4)
        self.add_vehicle(4, -10, 40, 0, 0, 0, 0, -2)
        self.add_vehicle(5, -10, 35, 0, 0, 0, 0, -2)
        self.add_vehicle(6, -10, 30, 0, 0, 0, 0, -2.2)
        self.add_vehicle(7, -5, 70, 0, 0, 0, 0, -8)

    def get_box_array(self):
        """
        Returns a TrackedOrientedBoxArray that contains the TrackedOrientedBoxes of all currently tracked vehicles.
        This is based on the true data, not on any noisy measurement data
        :return: TrackedOrientedBoxArray of the true position of all vehicles
        """
        h = SimulatedVehicle.create_def_header()  # create a default stamped header
        array = []  # Array of the boxes
        for v in self.vehicles:
            array.append(v.get_box())

        # Create the return value, i.e. the object of the correct type
        r = bobmsg.TrackedOrientedBoxArray(header=h, tracks=array)
        return r

    def get_gaussian_box_array(self, sd_pos=0.5, sd_vel=0.1, sd_angle=0.1, sd_lw=-1, cov_example_id=0):
        """
        Returns a TrackedOrientedBoxArray that contains noisy TrackedOrientedBoxes of all currently tracked vehicles.
        This data is based on noisy data, using a gaussian measurement error centered around the true data, with a
        standard deviation according to the parameters.
        A parameter < 0 causes this to return the true data for the respective entries.
        :param sd_pos: Standard deviation for the position (both x and y)
        :param sd_vel: Standard deviation for the velocity (in both x and y direction)
        :param sd_angle: Standard deviation for the angle of the vehicle
        :param sd_lw: Standard deviation for the length and width of the box.
        :param cov_example_id: The id of the example covariance to be used (imported from maven-1.bag)
        :return: TrackedOrientedBoxArray of the noisy position of all vehicles, using gaussian noise.
        """
        h = SimulatedVehicle.create_def_header()  # create a default stamped header
        array = []  # Array of the boxes
        for v in self.vehicles:
            array.append(v.get_gaussian_box(sd_pos=sd_pos, sd_vel=sd_vel, sd_angle=sd_angle, sd_lw=sd_lw,
                                            cov_example_id=cov_example_id))

        # Create the return value, i.e. the object of the correct type
        r = bobmsg.TrackedOrientedBoxArray(header=h, tracks=array)
        return r

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
        be passed to the TrackVisuals.plot_points_tuple function to display them.
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

    def get_gaussian_visual_list(self, stddev_pos=0.15, stddev_vel=0.05, color='r'):
        """
        Returns a list of positions that is suitable for display using a TrackVisuals object. The returned lists can
        be passed to the TrackVisuals.plot_points_tuple function to display them.
        This uses positions of the vehicles that are have gaussian noise
        :param stddev_pos: Standard deviation for the gaussian noise used for the position
        :param stddev_vel: Standard deviation for the gaussian noise used for the velocity
        :param color: The Color to be used, defaults to red
        :return: array of (x_pos, y_pos, ids, color) tuples that can be used for plotting
        """
        points = []
        for v in self.vehicles:
            x, y, vel_x, vel_y = v.get_sparse_gaussian_measurement(stddev_pos, stddev_vel)
            points.append((x, y, v.object_id, color))
        return points
