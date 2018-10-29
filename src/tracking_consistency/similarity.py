#!/usr/bin/env python
"""
File that implements functions/classes that are used for comparing two tracked objects wrt similarity.
A simple example for this is the pure distance
"""
import numpy as np
import rospy
from t2t_utils import *


def dist(p1, p2):
    """
    Takes two points as tuples (x,y) and returns the euclidean distance between the two points.
    :param p1: The first point as a tuple (x,y)
    :param p2: The first point as a tuple (x,y)
    :return: The distance between the two points
    """
    distance = np.linalg.norm(p1 - p2)
    return distance


class SimilarityChecker:
    """
    Class that implements methods for comparing two tracked objects wrt similarity.
    The methods of this class use hyperparameters (for example how to weight change in object angle vs change in pos.).
    Therefore, they cannot be easily implemented as static functions.

    Before using any of the functions, make sure that all relevant hyperparameters for this function are set to the
    desired values.

    All sim_<name> functions follow the same pattern:
    Parameters are old_obj and new_obj and time_diff, return is going to be a float that describes the similarity
    between the two objects that were passed as function of difference, therefore the same objects should return 0, and
    objects that are less similar return higher values.
    old_obj should always be the object that was tracked in the older time step (since for example the velocity of this
    object will be looked at)
    The objects should be in OrientedBox format. time_diff should be an int that describes how many
    time steps they were apart (0=same time step)
    """
    # List of Hyperparameters that will be used across the functions:
    # TODO find decent default values for all important ones
    dist_mult = 0.01  # How much velocity is weighted in the position comparison

    def __init__(self, dist_mult=0.01):
        self.dist_mult = dist_mult

    def sim_position(self, old_obj, new_obj, time_diff):
        """
        Compares two objects in OrientedBox format based on the positions. This takes into account velocity based on
        the following hyperparameter:
        - dist_mult: Multiplicative with velocity and time_diff to describe change in position

        This methods is basically already doing the thresholding: It draws a circle of size dist_mult*time_diff*velocity
        around the old_obj and returns 0 if new_obj is inside this circle (therefore 0 is a valid threshold for this
        functions similarity value). If new_obj is outside the circle, the distance to the circle is returned.
        :param old_obj: The first object in OrientedBox format
        :param new_obj: The second object in OrientedBox format
        :param time_diff: The numerical difference in time steps between the two objects, 0=same time step
        :return: A Float describing the similarity between the two objects as a function of difference (0 for the same)
        """
        phi = self.dist_mult
        # Calculate "full" velocity using pythagoras (since you only have x+y given)
        vel_x = np.abs(old_obj.velocity_x)
        vel_y = np.abs(old_obj.velocity_y)
        vel_full = np.sqrt(vel_x**2 + vel_y**2)
        # Calculate the euclidean distance between the two points
        p1 = (old_obj.center_x, old_obj.center_y)
        p2 = (new_obj.center_x, new_obj.center_y)
        pt_dist = dist(p1, p2)

        # Establish the "circle" size, i.e. the maximum distance that the two points can be apart to return a similarity
        # value of 0
        max_dist = time_diff * phi * vel_full

        if pt_dist <= max_dist:
            # The two objects were in acceptable distance to each other (based on velocity)
            return 0
        else:
            # The two objects were too far from each other, return the distance from the second one to the circle around
            # the first one
            return pt_dist - max_dist