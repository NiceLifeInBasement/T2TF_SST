#!/usr/bin/env python
"""
Adds all kind of utility functions for the t2tf algorithms that may be used in a variety of scenarios
"""
import numpy as np
import rospy


def extract_ids(boxes):
    """
    Takes an array of TrackedOrientedBoxes and extracts all unique ids from it, stored in a set
    :param boxes: An array of TrackedOrientedBoxes
    :return: A set of all ids in the boxes parameter
    """
    ids = set()
    for box in boxes:
        ids.add(box.object_id)
    return ids



