#!/usr/bin/env python
"""
First quick implementation of track to track association algorithms.
"""
import numpy as np
from bob_perception_msgs.msg import *
from math import isnan


def t2t_distance_single(track_i: TrackedOrientedBox, track_j: TrackedOrientedBox):
    """
    Takes two tracks in the form of TrackedOrientedBoxes and calculates the track to track distance between the two.
    This is only considering a single time step and no track history.
    Based on https://hal.archives-ouvertes.fr/hal-00740787/document
    :param track_i: TrackedOrientedBox representing the first track
    :param track_j: TrackedOrientedBox representing the second track
    :return: A float that specifies the t2t-distance between the two parameters.
    """
    # TODO implement the distance function
    return 0.0


def get_min_pos(dist_matrix):
    """
    Searches a matrix for its minimum value and returns this value and its position. If multiple entries are of the same
    (minimal) value, the position of the first is returned.
    :param dist_matrix: Matrix of floats that should be searched
    :return: (min_val, lin, col): Minimum value and its position (lin, col) in the matrix
    """
    # First flatten the matrix so that a min position can be found using min()
    # Not using the single line way for the sake of readability
    flat_list = []
    for sublist in dist_matrix:
        for item in sublist:
            flat_list.append(item)

    min_val = min(flat_list)  # Get the min value

    for lin in range(len(dist_matrix)):
        for col in range(len(dist_matrix[lin])):
            if dist_matrix[lin][col] == min_val:
                # min_value has been found in the matrix, return it and its position
                return min_val, lin, col


class TrackCluster:
    """
    This class represents a cluster of tracks for the track to track association algorithm.

    This class might be redundant, but can help grouping tracks and accessing them for track fusion.
    """
    tracks = []

    def __init__(self, tracks=None):
        if tracks is None:
            self.tracks = []
        else:
            self.tracks = tracks

    def add(self, track):
        self.tracks.append(track)

    def get(self):
        return self.tracks

    def size(self):
        return len(self.tracks)


def t2ta_collected(tracks, threshold, distance=t2t_distance_single):
    """
    First implementation of the Multisensor Track-To-Track Association Algorithm presented in the following paper:
    https://hal.archives-ouvertes.fr/hal-00740787/document.

    This version assumes that all sensor tracks have already been collected by the fusion center (and are passed as
    a single argument to this function). Management of different sensor tracks and synchronization issues needs to be
    done by the fusion center that calls this function. (--> _collected in the name)

    This version does not take into account track history. Therefore distance should be a function of type:
        (TrackedOrientedBox, TrackedOrientedBox) --> Float
    :param tracks: An array of TrackedOrientedBoxArrays. Each entry should contain the TrackedOrientedBoxArray that
                   represents the tracking of a single sensor in the current time step.
    :param threshold: The maximum distance threshold. The threshold symbolizes a gate out of which we assume that the
                      two tracks cannot originate from the same target
    :param distance: Function that calculates the distance between two given tracks. Should be of type:
                    (TrackedOrientedBox, TrackedOrientedBox) --> Float
    :return: TrackedOrientedBoxArray of the associated tracks for the current time step.
    """
    # This function follows the Algorithm 1 from https://hal.archives-ouvertes.fr/hal-00740787/document
    # Documentation follows the steps described
    # ----

    max_val = float("inf")  # The maximum value that a distance entry in the matrix can be set to
    nan = float("NaN")  # Not a Number stored in a variable for quick assignment

    # ----
    # 1) Collect the tracks of all the sensors.
    #       This is performed outside this function, and passed as the parameter tracks

    # 2) Assign a number from 1 to N to each track, N being the total number of tracks.

    # Count the total_no_tracks (==N) and create a list of all tracks
    # The number 1..N is the tracks position in the list of tracks (+1 due to the list first entry being 0)
    total_no_tracks = 0
    list_tracks = []  # A list of all tracks, assigning a number from 0..N-1 to all tracks
    for sensor_tracks in tracks:
        for single_track in sensor_tracks:
            total_no_tracks += 1  # Count how many tracks exist
            list_tracks.append(single_track)

    # 3) Create a NxN array for the TTTDs between the tracks

    # Create the distance matrix full of nan entries
    dist_matrix = [[nan for l in range(total_no_tracks)] for k in range(total_no_tracks)]

    # a) Set cells over the diagonal to a defined maximal value in  order  not  to  compute  twice
    #    the distance between two same tracks
    # THIS IS NOT NECESSARY, SINCE THE METHOD USED FOR b) ALSO SETS THE DIAGONAL TO max_val
    # Code that would be used if necessary:

    # for k in range(total_no_tracks):
    #    dist_matrix[k][k] = max_val

    # b) Set cells corresponding to two tracks of the same sensor to the maximum value,
    #    in order not to associate two tracks of a same sensor.

    n = 0  # The position in the overall matrix
    for sensor_tracks in tracks:
        start_n = n  # Position where this sensors track started
        end_n = start_n + len(sensor_tracks)  # Position where this sensors track ends
        # Loop twice over these start and end positions:
        # Example: 2 tracks from the sensor, start = 0, end = 2
        # Want to access [0][1], [1][0] and can access [0][0], [1][1]
        for pos_a in range(start_n, end_n):
            for pos_b in range(start_n, end_n):
                dist_matrix[pos_a][pos_b] = max_val
            # Increase n once per step of the outer loop, so it gets increased to the starting value of the next track
            n += 1

    # c) Set the remaining cells to the distance between the corresponding two tracks

    for pos_a in range(total_no_tracks):
        for pos_b in range(total_no_tracks):
            if isnan(dist_matrix[pos_a][pos_b]):
                # Value here is still nan, so it should be filled with a distance value
                d = distance(list_tracks[pos_a], list_tracks[pos_b])  # distance value between the two points
                dist_matrix[pos_a][pos_b] = d

    # d) Set cells where the distance is greater than a defined threshold to the maximum value.
    #    The threshold symbolizes a gate out of which we assume that the two tracks cannot
    #    originate from the same target

    for k in range(total_no_tracks):
        for l in range(total_no_tracks):
            if dist_matrix[k][l] > threshold:
                dist_matrix[k][l] = max_val

    # 4) Loop: Determine the minimal value (min_val) of the array and its position (lin,col) in the array.
    #    While min_val is smaller than max_val: Perform steps a - d
    min_val = float("-inf")  # Begin with min_val = negative infinity

    clusters = []  # List of all TrackCluster objects that represent the associated tracks
    # create the list that assigns a track to a cluster (where the number of that cluster >=0 and corresponds to its
    # position in the clusters array
    assignment = [nan for l in range(total_no_tracks)]

    # Implementation idea:
    #   Create a list of clusters
    #   Create a list that matches each entry in the matrix to an entry in the list of clusters
    #       a nan value in this list means that this entry has not been assigned to a cluster yet
    #   This allows for quick checking of the cluster a track belongs to
    #   And allows for some easy returning of the final clusters

    while min_val < max_val:
        # Pick min_val, (lin,col)
        min_val, lin, col = get_min_pos(dist_matrix)
        # add an additional break-condition that checks the while condition before all the calculations are done
        if not min_val < max_val:
            # Condition not fulfilled anymore, break the loop and go straight to step 5)
            break

        if isnan(assignment[lin]) and isnan(assignment[col]):
            # a) If none of the corresponding two tracks has not been inserted in a cluster yet,
            #    then put both of them in a new cluster.
            # Create a new cluster that contains only these two tracks
            new_cluster = TrackCluster(tracks=[list_tracks[lin], list_tracks[col]])
            pos_assignment = len(clusters)  # The position will be the last one (since its a new cluster)
            clusters.append(new_cluster)  # Add the cluster to the list of all clusters
            # Save the assignment number for these two tracks
            assignment[lin] = pos_assignment
            assignment[col] = pos_assignment
        elif isnan(assignment[lin] and not isnan(assignment[col])):
            # b) If only one has already been inserted in a cluster, then add the second to that cluster.
            # In this case lin has not been added yet, but col has been added
            cluster_no = assignment[col]  # Find out which cluster col has been assigned to
            # now add lin to this cluster, that also save the assignment in the list of assignments
            clusters[cluster_no].add(list_tracks[lin])
            assignment[lin] = cluster_no
        elif not isnan(assignment[lin] and isnan(assignment[col])):
            # b) If only one has already been inserted in a cluster, then add the second to that cluster.
            # In this case lin has been added , but col has not been added yet
            cluster_no = assignment[lin]  # Find out which cluster col has been assigned to
            # now add lin to this cluster, that also save the assignment in the list of assignments
            clusters[cluster_no].add(list_tracks[col])
            assignment[col] = cluster_no
        else:
            # c) If both have already been inserted in a cluster (the same or not), then do nothing
            pass

        # d) Set to max_val cells in the line lin and the ones in the column col, that correspond to tracks reported by
        #    the two concerned sensors
        # TODO d)

    # 5) Each track that has not been inserted in a cluster forms a new cluster (singletons)
    # TODO 5)
