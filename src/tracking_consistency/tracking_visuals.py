#!/usr/bin/env python
"""
This includes the TrackVisuals class, which can be used to display tracking data.
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
    Displays points by x/y coordinates, annotates them with an id (numeric) and allows for setting of color.

    Values are passed as three or four arrays (namely x, y, id, color) with color being optional. If no color is
    specified, the color that was set during init will be used (defaults to red). The arrays should be of the same size.

    For example, point 3 will be drawn at (x[3], y[3]) in color color[3], and will be annotated based on id[3]
    """

    fig = None  # Figure of the matplotlib graph
    ax = None  # Axes object in the graph
    ids = []  # List of the ids that are currently in use
    x = []  # List of x Coordinates
    y = []  # List of y Coordinates
    ann_list = []  # List of references to all annotations that are in use
    colormap = []  # List
    def_color = 'r'  # Default color, if no color map is given, this will be used for all points
    neg_limit = -50
    limit = 50

    def __init__(self, limit=50, neg_limit=-50, color='r'):
        """
        Initializes the TrackVisuals object.
        :param limit: The axis upper limit
        :param neg_limit: The axis lower limit
        :param color: The default color to be used if no colormap is passed while plotting
        """
        self.limit = limit
        self.neg_limit = neg_limit
        self.fig = plt.figure()
        self.ids = []
        self.ax = plt.axes(xlim=(self.neg_limit, self.limit), ylim=(self.neg_limit, self.limit))
        self.x, self.y = [], []
        self.def_color = color
        # self.ax.autoscale(enable=False)  # Appears to change nothing, should prevent axis autoscaling
        self.sc = self.ax.scatter(self.x, self.y, c=color)
        self.default_annotation()

    def plot_points(self, x_new, y_new, uid, color=[], append=False):
        """
        Plots tracking data in the graph, marking it with annotations of the corresponding UID.
        This resets all previously plotted data, use add_point if you want to add a single point.
        All parameter arrays should have the same size (they will be cut down to the size of the smallest one, excluding
        an empty color array)
        A color array can be specified, in this array every point gets assigned its color. If this is not specified or
        "[]" is passed, the default color that was specified during init will be used for all points. This does not
        affect annotation color.
        The append parameter allows you to specify whether you want to overwrite all currently drawn points. Appending
        points can be useful to plot data from different sources in the same plot. If the new color map is empty, the
        entire colormap will be reset as well.
        If you have multiple data sources, one should be set to append=False and the rest to append=True. Then, one will
        periodically wipe all data, and the others will add new information afterwards. Appending data without
        synchronized sources is not recommended due to this.
        :param x_new: The x position of the point as an array
        :param y_new: The y position of the point as an array
        :param uid: The numerical identifier of the point (which will be used as a label aswell)
        :param color: The colormap for this set. If not specified or "[]", the default color will be used for all points
        :param append: If True, the old points will not be deleted. If False, only the newly passed points will be shown
        """
        # The arrays are all cut down to the size of the smallest one that was passed
        # This always includes a check whether or not color was passed as an argument
        if len(color) > 0:
            min_length = min([len(x_new), len(y_new), len(uid), len(color)])
        else:
            min_length = min([len(x_new), len(y_new), len(uid)])
        if append:
            self.x.extend(x_new[0:min_length])
            self.y.extend(y_new[0:min_length])
            self.ids.extend(uid[0:min_length])
            if len(color) > 0:
                self.colormap = np.hstack([self.colormap, color[0:min_length]])
            else:
                self.colormap = []  # No colors were specified
        else:
            self.x = x_new[0:min_length]  # Expand list of points by the next point
            self.y = y_new[0:min_length]  # as above
            self.ids = uid[0:min_length]
            # Cut colors down in size if colors were specified
            if len(color) > 0:
                self.colormap = color[0:min_length]
            else:
                self.colormap = []  # No colors were specified

        # Remove annotations
        for i, a in enumerate(self.ann_list):
            a.remove()
        self.ann_list[:] = []  # clear the annotation list
        # Update the plot
        # You may also use: "self.sc.set_offsets(np.c_[self.x, self.y])", however this makes changing the colors harder

        # If the colormap is not empty (i.e. something was passed as an argument), change the colors as necessary
        self.clear_plot()  # necessary if you use ax.scatter to redraw the point
        if len(self.colormap) > 0:
            # Split up the array into multiple ones, where each one corresponds to a single color in the color map
            unique_colors = set(self.colormap)
            for next_color in unique_colors:
                # Get the indices of the current color
                next_positions = [i for i, j in enumerate(self.colormap) if j == next_color]
                # Select only the points of the next color
                next_x = [self.x[i] for i in next_positions]
                next_y = [self.y[i] for i in next_positions]
                # Plot these points using their corresponding color
                self.ax.scatter(next_x, next_y, c=next_color)
        else:
            # Else: use the default color
            self.ax.scatter(self.x, self.y, c=self.def_color)
        self.set_limits()  # Necessary because ax.scatter keeps autoscaling, even with autoscale=False

        # Add new annotations
        for i, txt in enumerate(self.ids):
            self.ann_list.append(self.ax.annotate(str(txt), (self.x[i], self.y[i])))

        # Pause for a very short time to let the graphics redraw
        plt.pause(0.0000001)

    # TODO consider adding a function that takes an array of 4-tuples (x,y,id,c) and passes this to plot_points

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
        """
        plt.xlim(self.neg_limit, self.limit)
        plt.ylim(self.neg_limit, self.limit)

    def get_ax(self):
        """
        Getter functin to get the ax object, if further manipulation of the graphic should be done outside of this class
        :return: The axis object of this TrackVisuals Objects
        """
        return self.ax
