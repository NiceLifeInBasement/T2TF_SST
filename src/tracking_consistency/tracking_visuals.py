#!/usr/bin/env python
"""
This visualizes laser scanner data
"""
import numpy as np
import rospy
from bob_perception_msgs.msg import *
import matplotlib.animation as plt_animation
import matplotlib.pyplot as plt


class TrackVisuals:
    """
    Class that stores relevant information regarding the visuals and has all methods to plot further data etc
    """

    fig = None  # Figure of the matplotlib graph
    ax = None  # Axes object in the graph
    ids = None  # A set of all ids that are in use
    x = []
    y = []

    def __init__(self):
        limit = 50
        neg_limit = (-1)*limit
        self.fig = plt.figure()
        self.ids = set()
        self.ax = plt.axes(xlim=(neg_limit, limit), ylim=(neg_limit, limit))
        self.x, self.y = [], []
        self.sc = self.ax.scatter(self.x, self.y, c='r')

    def plot_point(self, x_new, y_new, uid):
        """
        Plots a point in the graph, marking it with an annotation of the corresponding UID
        :param x_new: The x position of the point
        :param y_new: The y position of the point
        :param uid: The numerical identifier of the point (which will be used as a label aswell)
        :return:
        """
        # TODO DOESNT YET PLOT ANYTHING
        #   Basic idea: hold all point-pairs in the x/y arrays, update them via the plot function and periodically
        #   redraw the plot

        # print("Internal Visual at: "+str(x)+"|"+str(y))  # DEBUG
        self.x.append(x_new)  # Expand list of points by the next point
        self.y.append(y_new)  # see above
        self.sc.set_offsets(np.c_[self.x, self.y])
        plt.pause(0.00001)

    def add_point(self, x_new, y_new):
        self.x.append(x_new)  # Expand list of points by the next point
        self.y.append(y_new)  # see above

    def animate(self, i):
        # TODO if you append values to x/y here, they get plotted correctly

        # print("Animating: "+str(len(self.x)))
        self.sc.set_offsets(np.c_[self.x, self.y])
        # self.sc = self.ax.scatter(self.x, self.y, c='r')
        return [self.sc]

    def ani_fig(self):
        ani = plt_animation.FuncAnimation(self.fig, self.animate, interval=100, repeat=True)
        return ani

    def clear_plot(self):
        self.ax.clear()
