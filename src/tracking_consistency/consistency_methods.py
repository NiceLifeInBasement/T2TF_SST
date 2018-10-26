#!/usr/bin/env python
"""
Includes different methods for creating consistent tracking of the cars

"""
import numpy as np
import rospy


def dist(p1, p2):
    """
    Takes two points as tuples (x,y) and returns the euclidean distance between the two points.
    :param p1: The first point as a tuple (x,y)
    :param p2: The first point as a tuple (x,y)
    :return: The distance between the two points
    """
    distance = np.linalg.norm(p1 - p2)
    return distance


class ConsistencyTracker:
    """
    The ConsistencyTracker class can be used to improve consistency in a tracker.
    Has a working mode that determines how output is performed:
        plot: Plot the data. Requires to pass a TrackVisuals argument.
        publish: NOT YET IMPLEMENTED Publish the data to a python topic
        return: Only returns the data without doing anything special with it
    """
    old_data = None  # The data from the previous time step
    mode = ""
    visuals = None  # If mode=="plot" this should be a TrackVisuals Object used for visualization of the data
    phi = 0.1  # Multiplicative parameter for maximum distance calculation
    plot_color = 'b'
    append_data = True

    def __init__(self, working_mode="plot", radius_mult_constant=0.1, vis=None, plot_col='b', append=True):
        self.plot_color = plot_col
        self.mode = working_mode  # Store the working mode
        self.append_data = append
        self.phi = radius_mult_constant  # Set phi to the given constant value
        self.old_data = None
        self.visuals = vis

    def callback_consistency(self, data):
        """
        Callback method that subscribers can use to generate new data from this class. Will use the circle based
        consistency detection algorithm, and generate output according to the working_mode that was previously set.
        :param data: The data that was received from the topic in this time step
        :return: The newly generated data
        """
        # If this is the first timestep, store data and don't show anything yet
        # TODO alternatively, maybe just display information normally (instead of return)
        if self.old_data is None:
            self.old_data = data
            return

        # -------------------------------------------
        # ALGORITHM STEPS:
        # FOR ALL obj_old IN old_data:
        #   CALCULATE max_dist = phi*obj_old.velocity
        #   STORE old_pos = (obj_old.x, obj_old.y)
        #   STORE old_id = obj_old.id
        #   FOR ALL obj_now IN data:
        #       STORE now_pos = (obj_now.x, obj_now.y)
        #       CALCULATE d = dist(now_pos, old_pos)
        #       IF d < max_dist:
        #           obj_now.id = old_id
        # -------------------------------------------

        # Use the following to calculate velocity (since you only have x+y given)
        #   vel_x = np.abs(oriented_box.velocity_x)
        #   vel_y = np.abs(oriented_box.velocity_y)
        #   full_velocity = np.sqrt(vel_x**2 + vel_y**2)  # TODO check for correctness, currently just doing pythagoras

        # [... Code goes here ...]

        if self.mode == "plot":
            self.plot_data(data.boxes, self.visuals, self.plot_color, self.append_data)
        elif self.mode == "publish":
            print("Publishing is currently not supported")
        elif self.mode == "return":
            pass  # Do nothing, simply finish the function without doing anything special with the data
        else:
            print("Unsupported mode found, switching mode to \"plot\" but skipping this step")
            self.mode = "plot"

        self.old_data = data  # Store the data from the current time step for the next one as old data
        return data

    @staticmethod
    def plot_data(data, vis, color='b', append=False):
        """
        Takes a message of type TrackedOrientedBox[], a TrackVisuals Object and options for the plot, and extracts the
        necessary information from the data to plot the data.
        :param data: An array of TrackedOrientedBox objects (extracted from a msg from a relevant topic usually)
        :param vis: The TrackVisuals Object used to display the information
        :param color: A color, e.g. 'b', that will be used to display this information
        :param append: Whether or not to clear whats currently in the plot
        """
        x_pos = []
        y_pos = []
        uid = []
        for tracked_box in data:
            # Extract relevant information from every box (id, position)
            obj_id = tracked_box.object_id
            oriented_box = tracked_box.box
            # Append this information to the list of points for this step
            x_pos.append(oriented_box.center_x)
            y_pos.append(oriented_box.center_y)
            uid.append(obj_id)

        # Turn color into an array of adequate length. Cut the length, in case someone passed multiple colors:
        color_array = np.tile(color, len(x_pos))[0:len(x_pos)]
        # Update the plot with the list of points from this plot
        vis.plot_points(x_pos, y_pos, uid, color_array, append)

    # TODO create callback_publish which publishes the data instead of plotting it or add in a "mode"
