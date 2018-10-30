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
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def callback(data):
    global pub
    point_list = list(pc2.read_points(data.point_cloud, skip_nans=True, field_names=("x", "y", "z")))
    print("Acquired a List of points. Length: "+str(len(point_list)))
    pub.publish(data.point_cloud)


def republisher(topic_name):
    global pub
    rospy.init_node('laser_scan_extractor', anonymous=True)
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=100)
    rospy.Subscriber("tracked_objects/scan", TrackedLaserScan, callback)

    # and now start spinning so that it doesnt stop
    rospy.spin()


if __name__ == '__main__':
    topic_name = "pc2data"
    if len(sys.argv) > 1:
        bag_name = sys.argv[1]
    else:
        bag_name = "data/maven-1.bag"
    player_proc = subprocess.Popen(['rosbag', 'play', bag_name])
    republisher(topic_name)
    player_proc.terminate()

