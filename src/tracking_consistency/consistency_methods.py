#!/usr/bin/env python
"""
Includes different methods for creating consistent tracking of the cars

"""
from t2t_utils import *
import copy


# TODO assuming that data is coming in the TrackedLaserScan format
class ConsistencyTracker:
    """
    The ConsistencyTracker class can be used to improve consistency in a tracker.
    During init, this needs to be passed a function that can be used to evaluate similarity between two objects in the
    format of the methods of similarity.SimilarityChecker.sim_<...> Additionally, a threshold for this needs to be
    given, that will be used to compare these values to.
    Has a working mode that determines how output is performed:
        plot: Plot the data. Requires to pass a TrackVisuals argument.
        publish: NOT YET IMPLEMENTED Publish the data to a python topic
        return: Only returns the data without doing anything special with it
    The mode checker only checks if the name of the mode is contained in the set string, so mode=(plot+publish) would
    cause plotting and publishing to take place.
    """
    sim_function = None
    sim_threshold = 0
    old_data = None  # The data from the previous time step
    mode = ""
    visuals = None  # If mode=="plot" this should be a TrackVisuals Object used for visualization of the data
    phi = 1  # Multiplicative parameter for maximum distance calculation
    plot_color = 'b'
    append_data = True
    lost_ids = None  # Stores all ids that were lost over the last time steps
    ttl = 0  # How many time steps an id and its information is stored after getting lost
    id_map = None

    def __init__(self, sim_function, threshold, ttl=4, working_mode="plot", vis=None,
                 plot_col='b', append=True):
        # Set the args
        self.sim_function = sim_function
        self.sim_threshold = threshold

        # Set the kwargs
        self.ttl = ttl
        self.plot_color = plot_col
        self.mode = working_mode  # Store the working mode
        self.append_data = append
        self.old_data = None
        self.visuals = vis

        # Init the lost_ids + IDMapping data structure
        self.lost_ids = IDTracker()
        self.id_map = IDMapping()

    def callback_consistency(self, data):
        """
        Callback method that subscribers can use to generate new data from this class. Will use the set similarity based
        consistency detection algorithm, and generate output according to the working_mode that was previously set.
        :param data: The data that was received from the topic in this time step
        :return: The newly generated data
        """
        # TODO IMPORTANT: assumes that data is coming in in the TrackedLaserScan format (uses.boxes to get the arrays)

        # If this is the first time step, store data and don't show anything yet
        # TODO alternatively, maybe just display information normally (instead of return)
        if self.old_data is None:
            self.old_data = data
            return

        # ---
        # Clean up the data by applying the mapping of "wrong" ids to correct ids first
        #   i.e. apply knowledge gained in previous applications of the following algorithm
        #   otherwise, since the original tracker does not receive any update that was done here, the corrected ID
        #   would get corrupted again a single time step later
        # takes in data as an argument, and IDMapping got called earlier with every change in ID that was performed
        data = self.id_map.update_data(data)

        # ---
        # First, update the lost_ids data structure
        # 1. append all objects that were tracked in old_data, but not in data
        #   For this, use t2t_utils.extract_ids to get sets of ids in each of the data points:
        old_id_set = extract_ids(self.old_data.boxes)
        new_id_set = extract_ids(data.boxes)
        #   Now, remove all values from the old set that are also in the new set (old - new)
        diff_id_set = old_id_set.difference(new_id_set)
        #   These values specify all ids that were lost in the current time step
        #   For each of these values, add it and its respective box to the lost_ids data
        box_array = self.old_data.boxes
        for next_id in diff_id_set:
            for b in box_array:
                if next_id == b.object_id:
                    next_box = b.box
            self.lost_ids.add(next_id, next_box)

        # 2. increment the time on all of the ids (do this now, so that no 0s are stored that would imply the same step)
        self.lost_ids.time_step()
        # 3. trim the list wrt ttl
        self.lost_ids.trim(self.ttl)

        # ---
        # Then, compare all relevant values in lost_ids to data
        for pos in range(0, self.lost_ids.size()):
            # Compare this entry to all entries in the data array
            next_id = self.lost_ids.ids[pos]
            next_time = self.lost_ids.time[pos]
            next_box = self.lost_ids.boxes[pos]
            for i in range(0, len(data.boxes)):
                # Iterating over all entries in the data array
                # Calculate a similarity value based on the previously defined similarity function (see __init__)
                sim_value = self.sim_function(next_box, data.boxes[i].box, next_time)
                if sim_value <= self.sim_threshold:
                    # According to the similarity function, this object got its id changed
                    # TODO add something here that lets you later display this object in a different color (use id_map?)
                    # store this id in the mapping of wrong->correct ids for later use
                    self.id_map.add(data.boxes[i].object_id, next_id)
                    # Correct this id
                    data.boxes[i].object_id = next_id

        # ---
        # Now, perform output as necessary

        if "plot" in self.mode:
            self.plot_id_data(data.boxes, self.visuals, self.plot_color, self.append_data)
        elif "publish" in self.mode:
            print("Publishing is currently not supported")
        elif "return" in self.mode:
            pass  # Do nothing, simply finish the function without doing anything special with the data
        else:
            print("No mode found, switching mode to \"return\"")
            self.mode = "return"

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

    def plot_id_data(self, data, vis, color='b', append=False):
        # TODO turn this into an actual commented method

        # Quick fix: plot data that also checks the list of changed ids
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

        # ---
        # Go over list of changed uids and change the color to 'y'
        for oid in self.id_map.correct:
            if oid in uid:
                index = uid.index(oid)
                color_array[index] = 'y'

        vis.plot_points(x_pos, y_pos, uid, color_array, append)


class IDTracker:
    """
    Class that can be used to track IDs after they get lost (i.e. there is no further tracking done for them)
    This class stores the last existing OrientedBox for every id, as well as how long they were gone for.
    Includes methods to trim and update the stored objects
    """
    ids = []  # All ids that were lost recently
    time = []  # The time of how long the ids were stored already
    boxes = []  # List of OrientedBoxes of the objects in their last tracked states

    def __init__(self):
        pass

    def trim(self, ttl):
        """
        Removes all objects from the contained data that have been stored for longer than the ttl parameter
        :param ttl: Maximum time that objects may have been stored in the tracker
        :return: Number of removed objects
        """
        prev_length = len(self.ids)
        rem = []  # list of values to remove
        # Go over all entries and check them for time compared to ttl
        for pos in range(0, prev_length):
            if self.time[pos] > ttl:
                rem.append(pos)

        # Remove every entry that was found:
        #   Algorithm for this:
        #   create an empty array for every array where elements should be removed
        #   iterate over all entry positions in the array
        #       if the current position is not in the array specifying the position of elements to be removed:
        #           (--> the entry of this position should be kept and NOT removed)
        #           append this to the corresponding new array (initialized in the beginning)
        #   Replace the original arrays with the newly created ones
        rec_ids = []
        rec_time = []
        rec_boxes = []
        for x in range(0, len(self.ids)):
            if x not in rem and x < len(self.ids):
                rec_ids.append(self.ids[x])
                rec_time.append(self.time[x])
                rec_boxes.append(self.boxes[x])
        self.ids = rec_ids
        self.time = rec_time
        self.boxes = rec_boxes

        return len(rem)

    def time_step(self, steps=1):
        """
        Increase all time values by 1 (or the specified steps parameter if specified)
        :param steps: If more than 1 time_step should be performed, this parameter specifies how many should be done
        """
        self.time = [x+steps for x in self.time]  # Increment all values in time by steps

    def add(self, new_id, new_box):
        """
        Add a new object to the list.
        If the object was already in the list (going by id), it will instead get updated: the box is set to the new_box,
        and its time will be reset to 0. In that case, this function returns True.
        If the id is not yet in the list, it gets added instead (with time=0) and False is returned
        :param new_id: The id of the object
        :param new_box: The OrientedBox of the object
        :return: True, if the object was already in the list (going by id), else False
        """
        # The try-except block attempts to find the new_id in the list of ids. If it is found, the relevant values are
        # set to the new values. If it is not found, a ValueError will be thrown, and the values will get added to the
        # list instead
        try:
            index = self.ids.index(new_id)
            # id is obv. the same, so no change
            self.time[index] = 0  # reset the time to 0
            self.boxes[index] = new_box  # save the new box of the object instead of the old one
            return True
        except ValueError:
            # new_id was not yet found in ids, append the object instead
            self.ids.append(new_id)
            self.time.append(0)
            self.boxes.append(new_box)
            return False

    def size(self):
        """
        Returns the total number of elements in the data structure.
        :return: The number of elements stored in the data structure.
        """
        return len(self.ids)


class IDMapping:
    """
    Class that implements a mapping of two arrays of ids to each other. This allows to store information about corrected
    ids (after an id was corrected, simply add it to an IDMapping with the previous(wrong) and new(correct) ID.
    The class provides methods to clean up data by applying the stored mapping
    """
    wrong = []
    correct = []

    def __init__(self):
        self.wrong = []
        self.correct = []

    def add(self, wr, cor):
        # TODO consider adding methods to prevent adding the same wrong id multiple times (correct multi should be ok)
        self.wrong.append(wr)
        self.correct.append(cor)

    def update_data(self, data):
        corrected_data = copy.deepcopy(data)
        for box in corrected_data.boxes:
            try:
                # Try to find the id of the next box in the list of ids that were already detected to be wrong
                index = self.wrong.index(box.object_id)
                # Found an index for the id the list of wrong ids, so replace it
                box.object_id = self.correct[index]
            except ValueError:
                pass  # id of the box not found in the set of wrong ids
        return corrected_data
