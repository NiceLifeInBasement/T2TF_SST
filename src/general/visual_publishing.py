"""
Includes messages for publishing visualization of the results
"""
import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

def boxes_to_markerArray(boxes, color):
    """
    Converts a set of tracked oriented boxes to a markerarray and returns that
    :param boxes: Array of TrackedOrientedBoxes
    :param color: Color to be used, should be a tuple (rgba)
    :return: A MarkerArray
    """
    marker_array = MarkerArray()

    point_array = []

    # http://wiki.ros.org/rviz/DisplayTypes/Marker#The_Marker_Message
    # TODO implement this function
    # TODO this should return a marker array
    #   then in a higher level function, the marker arrays of multiple messages should be combined (see stub below)
    #   and that should be published to a topic which rviz subs to
    #   --> result should be a plot of the markers in rviz
    #   type == point probably
    #   create Markers one by one, and append them to the marker array

    return marker_array


def merge_markerArray(array_A, array_B):
    """
    Takes two MarkerArrays A and B and returns a merged version of the two
    :param array_A: First MarkerArray
    :param array_B: Second MarkerArray
    :return: Merge of A and B
    """
    # TODO implement this stub
    pass
