#!/usr/bin/env python
"""
This file starts reading data (and playing a bagfile) and takes data from the TrackedLaserScan and re-publishes it as
point-cloud data to another topic (therefore making it readable to tools such as RViz)
"""
import numpy as np
import rospy
from roslib import message
from collections import namedtuple
import sys
import subprocess
from bob_perception_msgs.msg import *
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg


def callback_modded(data):
    global pub, xoff, yoff
    divide = True  # Just a flag for me
    multi = 1  # Multiply all values by this

    # ---- #
    # Actual Function
    point_list = list(pc2.read_points(data.point_cloud, skip_nans=True, field_names=("x", "y", "z")))
    print("Acquired a List of points. Length: "+str(len(point_list))+" || Entry0: "+str(point_list[0]))

    # Currently trying to normalize the data in the array to the position of one of the entries
    # Struggling with the pc2 -> array -> pc2 conversion though
    # Probably a waste of time anyway, since 90-110pts is not enough to display any meaningful data
    # and the maven-X.bag files have only that amount of points.
    # In general, the fact that the points are at x/y-values like x.e+07 is kinda weird
    # According to some quick research, rviz has some known issues with this kind of size of values

    if len(point_list)>0 and xoff == 0:
        xoff = point_list[0][0]
    if len(point_list) > 0 and yoff == 0:
        yoff = point_list[0][1]
    redone_list = []
    min_x = 999999999999999999
    max_x = -min_x
    for t in point_list:
        if divide:
            new_t = ((t[0]-xoff) * multi, (t[1]-yoff) * multi, t[2])
        else:
            new_t = (t[0], t[1], t[2])
        redone_list.append(new_t)
        if new_t[0] < min_x:
            min_x = new_t[0]
        if new_t[0] > max_x:
            max_x = new_t[0]
    print("\tmin_x: "+str(min_x)+" / max_x: "+str(max_x))

    c = pc2.create_cloud(data.point_cloud.header, data.point_cloud.fields, redone_list)
    # c = data.point_cloud
    pub.publish(c)


def callback(data):
    # Simply publish the pc2 data without any modifications at all
    global pub
    cloud = data.point_cloud
    # cloud.header = data.header  # Header of cloud is not set, so copy the original header (from the data)
    # cloud.header.frame_id = "gps_antenna"
    pub.publish(cloud)


def republisher(topic_name):
    global pub
    rospy.init_node('laser_scan_extractor', anonymous=True)
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=10)
    rospy.Subscriber("tracked_objects/scan", TrackedLaserScan, callback)

    # and now start spinning so that it doesnt stop
    rospy.spin()


if __name__ == '__main__':
    global xoff, yoff
    # these offset from maven-1.bag dont really work aswell, since the original data has data in a completly different
    # region (these offsets are for the c2x data not the trackedlaserscan)
    xoff = 470224.387970
    yoff = 5522444.309280
    # set xoff/yoff to 0 if you want to grab them from the first data point
    topic_name = "pc2data"
    if len(sys.argv) > 1:
        bag_name = sys.argv[1]
    else:
        bag_name = "data/maven-1.bag"
    player_proc = subprocess.Popen(['rosbag', 'play', bag_name])
    republisher(topic_name)
    player_proc.terminate()

