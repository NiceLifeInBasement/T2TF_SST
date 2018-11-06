#!/usr/bin/env python
"""
Contains class definitions for data simulation for the algorithms
"""
import rospy
import numpy as np
import bob_perception_msgs.msg as bobmsg
from std_msgs.msg import Float64MultiArray, Float64, MultiArrayLayout, MultiArrayDimension
import std_msgs.msg


class SimulatedVehicle:

    # The following class variables are the core data that is needed for the TrackedOrientedBox
    object_id = -1
    real_center_x = 0
    real_center_y = 0
    real_angle = 0
    real_length = 0
    real_width = 0
    real_velocity_x = 0
    real_velocity_y = 0

    # The following are general class variables
    # [...]

    def __init__(self, oid, x=0.0, y=0.0, angle=0.0, length=0.0, width=0.0, vel_x=0.0, vel_y=0.0):
        """
        Initialize the Simulated Vehicle with its core attribute starting values
        :param oid: The object_id
        :param x: The x position
        :param y: The y position
        :param angle: The angle, less important than x/y and velocity
        :param length: length of the box, can be disregarded in most cases
        :param width: width of the box, can be disregarded in most cases
        :param vel_x: Velocity in x direction
        :param vel_y: Velocity in y direction
        """
        self.object_id = oid
        self.real_center_x = float(x)
        self.real_center_y = float(y)
        self.real_angle = float(angle)
        self.real_length = float(length)
        self.real_width = float(width)
        self.real_velocity_x = float(vel_x)
        self.real_velocity_y = float(vel_y)

    def get_box(self):
        """
        Creates and returns the TrackedOrientedBox for this vehicle
        :return: Information about this object in the TrackedOrientedBox format
        """
        header = self.create_def_header()
        # ----
        # CURRENTLY THIS USES THE DEFAULT SETTING FOR COV: ONLY CENTER AND VELOCITY
        cov_center = True
        cov_angle = False
        cov_lw = False
        cov_vel = True
        # ----
        oriented_box = bobmsg.OrientedBox(header=header, center_x=self.real_center_x, center_y=self.real_center_y,
                                          angle=self.real_angle, length=self.real_length, width=self.real_width,
                                          velocity_x=self.real_velocity_x, velocity_y=self.real_velocity_y,
                                          covariance=self.get_def_cov(), covariance_center=cov_center,
                                          covariance_angle=cov_angle, covariance_length_width=cov_lw,
                                          covariance_velocity=cov_vel)

        return bobmsg.TrackedOrientedBox(object_id=self.object_id, box=oriented_box)

    def create_def_header(self):
        """
        Creates a default header that can be used when creating objects for msgs.
        :return: The default header
        """
        # Header can have seq = 0
        # should have working time stuff (not sure how to implement this, maybe in early tests just put 0)
        # frame_id = "ibeo_front_center"
        h = std_msgs.msg.Header()
        # h.stamp = rospy.Time.now()  # rospy.init_node() needs to be called before this works
        # Check if you need to stamp this, and if yes, if this works
        h.frame_id = "ibeo_front_center"
        return h

    def get_def_cov(self):
        """
        Creates a default covariance matrix that has values for center and velocity
        :return: The covariance matrix in format std_msgs/Float64MultiArray
        """
        # EXPLANATION OF THE COVARIANCE MATRIX:
        # Covariance of the seven dimensional state vector [center_x, center_y, angle,
        # length, width, velocity_x, velocity_y]. Some elements may be set to NaN if
        # they have not been computed.
        #
        # Example of a covariance matrix where covariance_center = true,
        # covariance_velocity = true and all other covariance flags set to false:
        #
        #              | 1.0, 0.1, nan, nan, nan, 0.2, 0.2 |
        #              | 0.1, 2.0, nan, nan, nan, 0.2, 0.2 |
        #              | nan, nan, nan, nan, nan, nan, nan |
        # covariance = | nan, nan, nan, nan, nan, nan, nan |
        #              | nan, nan, nan, nan, nan, nan, nan |
        #              | nan, nan, nan, nan, nan, nan, nan |
        #              | 0.2, 0.2, nan, nan, nan, 3.0, 2.0 |
        #              | 0.2, 0.2, nan, nan, nan, 2.0, 4.0 |

        nan = float("NaN")  # nan variable since it gets used quite a bit
        w, h = 7, 7  # Matrices will always be 7x7, filled with NaN as init
        cov_array = [[nan for x in range(w)] for y in range(h)]

        # Now insert sensor covariance data into this, for the default case: center+velocity
        # currently just 1.0 for all values
        # Top left block:
        cov_array[0][0] = 1.0
        cov_array[1][0] = 1.0
        cov_array[0][1] = cov_array[1][0]  # Symmetric
        cov_array[1][1] = 1.0
        # Bottom right block:
        cov_array[5][5] = 1.0
        cov_array[6][5] = 1.0
        cov_array[5][6] = cov_array[6][5]
        cov_array[6][6] = 1.0
        # First of the outer block (top right):
        cov_array[0][5] = 1.0
        cov_array[0][6] = 1.0
        cov_array[1][5] = 1.0
        cov_array[1][6] = 1.0
        # bottom left block is a copy of that
        cov_array[5][0] = cov_array[0][5]
        cov_array[6][0] = cov_array[0][6]
        cov_array[5][1] = cov_array[1][5]
        cov_array[6][1] = cov_array[1][6]
        # TODO check that all the symmetry is correct and maybe change the values to something other than 1.0
        # Float64MultiArray doesnt work with 2D Arrays, so the array needs to be flattened:
        cov_array_flat = []
        # (not using a one liner for the sake of readability)
        for sublist in cov_array:
            for item in sublist:
                cov_array_flat.append(item)

        # The following was extracted from maven-1.bag
        dim_0 = MultiArrayDimension(label="", size=7, stride=49)
        dim_1 = MultiArrayDimension(label="", size=7, stride=7)
        cov_dim = [dim_0, dim_1]
        cov_layout = MultiArrayLayout(dim=cov_dim, data_offset=0)

        return Float64MultiArray(layout=cov_layout, data=cov_array_flat)
        # return Float64MultiArray(data=[0.0])

    def get_sparse_gaussian_measurement(self, stddev_pos=0.15, stddev_vel=0.05):
        """
        Gets a sparse measurement of the vehicle, that has a gaussian error.
        Sparse means that only the position (x,y) and the velocity (vel_x, vel_y) are returned.
        Gaussian noise std. deviation can be controlled for position and velocity (independent of each other)
        :param stddev_pos: Standard deviation for the gaussian noise used for the position
        :param stddev_vel: Standard deviation for the gaussian noise used for the velocity
        :return: noisy x, y, velocity_x, velocity_y
        """
        x = np.random.normal(loc=self.real_center_x, scale=stddev_pos)
        y = np.random.normal(loc=self.real_center_y, scale=stddev_pos)
        vel_x = np.random.normal(loc=self.real_velocity_x, scale=stddev_vel)
        vel_y = np.random.normal(loc=self.real_velocity_y, scale=stddev_vel)
        return x, y, vel_x, vel_y

    def basic_move(self, steps=1):
        """
        Perform one or multiple steps along the current velocity on x and y, without changing anything else about the
        vehicle. One step is equivalent to moving along the x/y axis once according to the x/y velocity
        :param steps: How many steps should be performed.
        :return: x, y coordinates of the new position
        """
        self.real_center_x += self.real_velocity_x * steps
        self.real_center_y += self.real_velocity_y * steps
        return self.real_center_x, self.real_center_y

    def basic_accelerate(self, factor, factor_y=None):
        """
        Increase (or decrease) the velocity via multiplication with a floating points factor.
        The factor is the same for x and y vel. by default, but if the factor_y argument is passed, two different
        values can be used
        :param factor: The multiplicative factor for the velocity
        :param factor_y: If None, the factor parameter will be used for x and y velocity. If a value is passed, this
        value will be used for the y velocity, and the factor parameter will be used for x vel.
        :return:
        """
        self.real_velocity_x *= factor
        if factor_y is None:
            self.real_velocity_y *= factor
        else:
            self.real_velocity_y *= factor_y
