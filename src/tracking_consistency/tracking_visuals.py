#!/usr/bin/env python
"""
This visualizes laser scanner data
"""
import numpy as np
import pandas as pd
import rospy
from bob_perception_msgs.msg import *
import matplotlib.animation as plt_animation
import matplotlib.pyplot as plt


class TrackVisuals:
    """
    Class that stores relevant information regarding the visuals and has all methods to plot further data etc.
    Use this to visualize tracking data.
    Displays points by x/y coordinates, annotates them with an id (numeric) and allows for setting of color
    """

    fig = None  # Figure of the matplotlib graph
    ax = None  # Axes object in the graph
    ids = None  # A set of all ids that are in use
    x = []  # List of x Coordinates
    y = []  # List of y Coordinates
    ann_list = []  # List of references to all annotations that are in use
    colormap = []
    def_color = 'r'  # Default color
    neg_limit = -50
    limit = 50

    def __init__(self, limit=50, neg_limit=-50, color='r'):
        self.limit = limit
        self.neg_limit = neg_limit
        self.fig = plt.figure()
        self.ids = set()
        self.ax = plt.axes(xlim=(self.neg_limit, self.limit), ylim=(self.neg_limit, self.limit))
        self.x, self.y = [], []
        self.def_color = color
        # self.ax.autoscale(enable=False)  # Appears to change nothing, should prevent axis autoscaling
        self.sc = self.ax.scatter(self.x, self.y, c=color)
        self.default_annotation()

    def plot_points(self, x_new, y_new, uid, color=[]):
        """
        Plots a point in the graph, marking it with an annotation of the corresponding UID. This resets all previous
        points, use add_point if you want to add a single point
        :param x_new: The x position of the point as an array
        :param y_new: The y position of the point as an array
        :param uid: The numerical identifier of the point (which will be used as a label aswell)
        :param color: The colormap for this set
        :return:
        """
        self.x = x_new  # Expand list of points by the next point
        self.y = y_new  # as above
        self.ids = uid
        self.colormap = color

        # TODO the plotting of the colors via c=colormap does not yet work, maybe switch back to updating the plot
        # TODO instead of drawing a new scatter every time

        # Remove annotations
        for i, a in enumerate(self.ann_list):
            a.remove()
        self.ann_list[:] = []  # not sure,
        # Update the plot
        # You may also use: "self.sc.set_offsets(np.c_[self.x, self.y])", however this makes changing the colors hard
        # If the colormap is not empty (i.e. something was passed as an argument), change the colors as necessary
        self.clear_plot()
        if len(self.colormap) > 0:
            self.ax.scatter(self.x, self.y, c=self.colormap)
            self.set_limits()  # Necessary because ax.scatter keeps autoscaling, even with autoscale=False
        else:
            # Else: use the default color
            self.ax.scatter(self.x, self.y, c=self.def_color)
            self.set_limits()  # Necessary because ax.scatter keeps autoscaling, even with autoscale=False

        # Add new annotations
        for i, txt in enumerate(self.ids):
            self.ann_list.append(self.ax.annotate(str(txt), (self.x[i], self.y[i])))

        # Pause for a very short time to let the thread redraw
        plt.pause(0.0000001)

    def add_point(self, x_new, y_new):
        """
        Add a point to the display, without any annotation.
        Since this clutters the plot, it should only be used for testing purposes.
        :param x_new: The x coordinate of the new point
        :param y_new: The y coordinate of the new point
        """
        self.x.append(x_new)  # Expand list of points by the next point
        self.y.append(y_new)  # see above
        self.sc.set_offsets(np.c_[self.x, self.y])
        plt.pause(0.00001)

    def clear_plot(self):
        """
        Clear the plot
        :return:
        """
        self.ax.clear()

    def default_annotation(self):
        """
        Sets up default annotation for the plot
        """
        self.ax.set_xlabel("X Coordinate of the Object")
        self.ax.set_ylabel("Y Coordinate of the Object")
        self.ax.set_title("Visualization of tracked objects")

    def set_limits(self):
        """
        Set the x and y limits on the axis to the set values.
        Necessary if some function calls starts messing with the desired values.
        :return:
        """
        plt.xlim(self.neg_limit, self.limit)
        plt.ylim(self.neg_limit, self.limit)
