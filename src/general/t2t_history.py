#!/usr/bin/env python
"""
Contains:
    TrackingHistory class that stores all historic tracks for a (sensor_id, object_id)
    t2tdistance functions that include history in their calculations

All this needs to be used by the t2ta_historic function that is implemented in t2ta_algorithms.py

TODO insert t2th class and two t2td functions (single time step and the one summing over it) along with helper funcs
"""
import rospy
import numpy
from t2ta_algorithms import *
from bob_perception_msgs.msg import *
import sys

class TrackingHistory:
    """
    TrackingHistory class that stores all historic tracks for a (sensor_id, object_id)
    """
    sensor_id_list = []  # List of strings of sensor ids

    # Data is a 3D List in the following format:
    # data is a list of the sensors, for example the first entry is the entry for the first entry in the sensor ids list
    # every entry in this list is then a list of all time steps where data from this sensor was received
    #   these vary between sensors
    # every entry in this list of observations is an array of TrackedOrientedBoxes that represent the sensors
    # measurement at this time step.
    # These are not sorted in any way, they simply represent what the sensor returned
    # This means it can be accessed in the following way:
    # data[sensor_id][timestep][no_track] where:
    #   sensor_id : The numerical id of the sensor (= the position of the nam in sensor_id_list)
    #   timestep : -1 to acquire the most recent one, 0 to acquire the first one, etc
    #   no_track : Which track of the measurement should be looked at
    # Keep in mind that no_track is not equal to object id, usually you will be iterating over all tracks like this:
    #   for track in data[sensor_id][timestep]:
    #       if track.object_id==wanted_object_id:
    #           # use track.box here ...
    # or something similar
    data = []

    def __init__(self):
        sensor_id_list = []  # List of strings of sensor ids
        data = []

    def new_sensor(self, sensor_name):
        """
        Appends a new sensor to the list of known sensors, and extends the data by a new empty array where measurements
        for this sensor will be stored
        :param sensor_name: The unique name of the sensor
        :return: True, if the sensor was appended, False if it already existed
        """
        if sensor_name in self.sensor_id_list:
            # name is already in the list (i.e. already in use)
            # don't add anything, instead just return False
            return False
        self.sensor_id_list.append(sensor_name)
        self.data.append([])
        return True

    def is_known(self, sensor_name):
        """
        Checks if a given sensor is known in the tracking history, without adding anything to the data structure if the
        given name is not known
        :param sensor_name: The unique name of the sensor
        :return: True, if the sensor is already known, else false
        """
        return sensor_name in self.sensor_id_list

    def tracking_steps(self, sensor_name):
        """
        Returns how many time steps were tracked for the given sensor, or -1 if the sensor is not known.
        This is equivalent to the length of the array that stores all measurements for this sensor
        :param sensor_name: The unique name of the sensor
        :return: How many time steps were tracked for the given sensor, or -1 if the sensor is not known.
        """
        if not self.is_known(sensor_name):
            return -1
        return len(self.data[self.sensor_id_list.index(sensor_name)])

    def add(self, sensor_name, measurement):
        """
        Adds a new measurement to the tracking history, based on the sensor name.
        If the sensor is not yet known, it will be added to the list of sensors
        :param sensor_name: The unique name of the sensor that produced this measurement
        :param measurement: An array of TrackedOrientedBoxes that represent the most recent measurement of the sensor.
        If this is empty, nothing will be added
        :return: True, if the measurement was successfully added, else false.
        """
        if len(measurement) == 0:
            return False
        if not self.is_known(sensor_name):
            self.new_sensor(sensor_name)
        sensor_id = self.sensor_id_list.index(sensor_name)  # The numerical id of the sensor
        # Append the measurement to the list of tracks for this sensor_id
        self.data[sensor_id].append(measurement)
        return True

    def get_absolute(self, object_id, sensor_name, time=-1, hist_size=1):
        """
        Acquire tracks for the set of identification parameters.
        A hist_size of 1 means only a single track will be returned.

        This is based on absolute timesteps, that means an array with a length equivalent to hist_size will be returned.
        Due to this, sensors with different frequencies may cause issues when using this. For example: Sensor A sends 5x
        as many measurements as Sensor B. Giving a hist_size of 5 will now return 5 measurements of sensor A that all
        fit to the most recent measurement of Sensor B, but also 4 more measurements for B that don't have any match in
        the list of measurements from sensor A.
        :param object_id: The object_id of the vehicle to be analyzed
        :param sensor_name: The name of the sensor that tracked the vehicle to be analyzed
        :param time: The time for which the data should be returned. A time of -1 means that the most recently added
        tracks will be used. If a value that is greater than the number of measurements for this sensor is given, then
        it defaults to -1. -1 therefore is equivalent to giving (tracking_steps(sensor_name)-1)
        :param hist_size: How far into the past the tracks should go. A value of 1 means a single track will be returned
        (the one for time step time). If hist_size is too big, the maximum possible value will be used instead.
        :return: A list of TrackedOrientedBoxes for this set of identification parameters. The first object in this list
        will be the most recent one, and the last object in the list will be the one that was measured first (respecting
        the history size and the given time index)
        :raises: ValueError if the sensor name is not known
        """
        # --- First, go over parameters and correct them as necessary

        if not self.is_known(sensor_name):
            raise ValueError("Attempting to acquire data for unknown sensor \""+sensor_name+"\"!")
        sensor_id = self.sensor_id_list.index(sensor_name)  # The numerical id of the sensor
        if self.tracking_steps(sensor_name) <= time:
            # time is too big, set it to -1 to use the last track instead
            time = -1
        # Check hist_size. If a timepoint is specified, max_hist_size is one bigger than it (since time is 0-indexed and
        # history_size is 1-indexed). If -1 is specified, reset time to the maximum possible value now (which is equiv.)
        if time == -1:
            time = self.tracking_steps(sensor_name) - 1
        max_hist_size = time+1
        if hist_size > max_hist_size:  # If hist_size exceeds the upper limit, set it to this upper limit
            hist_size = max_hist_size

        # --- Finished preparing all parameters, now extract all tracks for this:

        historic_tracks = []  # The list of tracks including the history that will be returned
        sensor_tracks = self.data[sensor_id]  # All tracks of this sensor
        for i in range(hist_size):
            # go over all positions in the tracking array that fit the parameters
            measurement = sensor_tracks[time-i]  # Acquire the measurement for this time step
            # The measurement contains the oriented boxes for all objects that were tracked by the sensor at that time
            for oriented_box in measurement:  # Go over all objects in the measurement
                if oriented_box.object_id == object_id:  # If the object has a matching id, add it to the historic track
                    historic_tracks.append(oriented_box)
                    break  # Only add one box per measurement in case multiple boxes with the same id are contained

        return historic_tracks

    def get_timed(self, object_id, sensor_name, time=-1, hist_size=rospy.Duration(0)):
        """
        Acquire tracks for the set of identification parameters.
        A hist_size of 1 means only a single track will be returned.

        This is not based on a number of time steps, but instead returns a dynamic number of time steps depending on
        how far behind the base time step (which is the one matching the time parameter) they are. For example,
        giving time=-1 and hist_size=rospy.Duration(1) will return all measurements that have a time stamp of at most
        1 second before the time stamp of the newest measurement.

        The data resulting from this will likely require further processing, since acquiring data for the same
        parameters but different sensor_names will result in arrays of different length, which are therefore not
        directly compatible with the t2t_distance function.
        :param object_id: The object_id of the vehicle to be analyzed
        :param sensor_name: The name of the sensor that tracked the vehicle to be analyzed
        :param time: The time for which the data should be returned. A time of -1 means that the most recently added
        tracks will be used. If a value that is greater than the number of measurements for this sensor is given, then
        it defaults to -1. -1 therefore is equivalent to giving (tracking_steps(sensor_name)-1)
        :param hist_size: A rospy.Duration that specifies how far into the past this should go. If time is Duration(0)
        (the default value), then only the measurement at time step "time" will be returned (still in array though!)
        :return: A list of TrackedOrientedBoxes for this set of identification parameters. The first object in this list
        will be the most recent one, and the last object in the list will be the one that was measured first (respecting
        the history size and the given time index). The size of this is determined by hist_size, so that the first and
        last object are at most hist_size (which is a rospy.Duration object) time apart.
        """
        # --- First, go over parameters and correct them as necessary

        if not self.is_known(sensor_name):
            raise ValueError("Attempting to acquire data for unknown sensor \"" + sensor_name + "\"!")
        sensor_id = self.sensor_id_list.index(sensor_name)  # The numerical id of the sensor
        if self.tracking_steps(sensor_name) <= time:
            # time is too big, set it to -1 to use the last track instead
            time = -1
        # If -1 is specified for time, reset time to the maximum possible value now (which is equiv.)
        if time == -1:
            time = self.tracking_steps(sensor_name) - 1

        # --- Finished preparing all parameters, now extract all tracks for this:

        sensor_tracks = self.data[sensor_id]  # All tracks of this sensor

        historic_tracks = []  # The list of tracks including the history that will be returned
        # Acquire a list of all tracks

        # Acquire the base time, which is the timing of the measurement performed at time step "time"
        # Can just take the first entry of the measurement, because the timing across a single measurement should be
        # the same for all objects, regardless of their id
        base_time = sensor_tracks[time][0].box.header.stamp

        keep_going = True  # Flag that determines if the data is still fitting the time constraint.
        for i in range(time, 0, -1):
            #  i decreases every iteration
            # add the measurement for this iteration to the batch (if it contained a time and object id matching track)
            measurement = sensor_tracks[i]
            # The measurement contains the oriented boxes for all objects that were tracked by the sensor at that time
            for oriented_box in measurement:  # Go over all objects in the measurement
                if oriented_box.object_id == object_id:  # If the object has a matching id, add it to the historic track
                    # but first, check its time stamp
                    if base_time - oriented_box.box.header.stamp <= hist_size:
                        historic_tracks.append(oriented_box)
                    else:
                        keep_going = False
                    break  # Only add one box per measurement in case multiple boxes with the same id are contained
            # Looped over all oriented_boxes in the measurement checking their obj_id, now check if the time constraint
            # was still fulfilled after the last box was appended
            if not keep_going:
                break

        return historic_tracks


def closest_match(data, stamp):
    """
    Selects the closest match wrt time from the list data compared to the timestamp stamp
    :param data: A list of objects with a header that have a time stamp
    :param stamp: Timestamp to compare to
    :return: The object from the list with the closest matching time stamp
    """
    if data is None or stamp is None or len(data) == 0:
        return None  # return None if no data was given
    min_val = rospy.Duration(99999999, 0)  # init value is a very long duration, so that the first comp is smaller
    min_pos = 0  # position of the minimum
    for i in range(len(data)):  # use a counter var to store min position
        datapoint = data[i]  # select the current data point from the list
        diff = datapoint.header.stamp - stamp
        diff.secs = abs(diff.secs)
        diff.nsecs = abs(diff.nsecs)
        if diff < min_val:
            min_val = diff
            min_pos = i
    # print("selected"+str(min_pos)+"/"+str(len(data))+" data with diff: "+
    #      str(min_val.secs)+" nsecs:"+str(min_val.nsecs))
    return data[min_pos]


def common_history(tracks):
    """
    Takes a list of tracks, which in turn are lists of TrackedOrientedBoxes. These lists are assumed to be generated by
    TrackingHistory.get_timed, and therefore have different sizes. This function converts all these into a list of best
    fitting tracks, with a maximum size (i.e. the size of the smallest track).
    For this, the smallest track is iterated, and for each time step the closest match w.r.t. its time step from all
    other tracks is selected
    :param tracks: A list of tracks, which are lists of TrackedOrientedBoxes. Can be produced by
        TrackingHistory.get_timed. These explicitly don't need to have the same length.
    :return: A list of tracks, which are lists of TrackedOrientedBoxes. These tracks will have the same length, and
    their content will be chosen based on time stamps. The new length will be the maximum possible length, which is the
    length of the smallest track
    """
    if len(tracks) < 2:
        raise ValueError("Need at least 2 tracks to build a common history between them!")
    # First, find the size that all tracks will have at the end, as well as the position of the smallest track
    final_length = sys.maxsize  # Starting value as big as possible
    smallest_track = 0  # Starting value is 0
    pos = -1  # tracking var
    for track in tracks:
        pos += 1
        if len(track) < final_length:
            final_length = len(track)
            smallest_track = pos

    # Found smallest track and its length, proceed by iterating over this track
    adjusted_tracks = [[] for w in tracks]  # The new list of tracks after they were adjusted to fit to each others time
    for entry in tracks[smallest_track]:
        # For every entry, select the closest match that fits it from each track and append it to the adjusted track
        pos = 0
        timestamp = entry.box.header.stamp
        for track in tracks:
            # go over this track, and find the closest match
            match = closest_match(track, timestamp)
            # append this closest match to its respective list
            adjusted_tracks[pos].append(match)
            # increase the position counter that goes over all tracks
            pos += 1
    return adjusted_tracks
