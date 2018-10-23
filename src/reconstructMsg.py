#!/usr/bin/env python

import rosbag
import sys

#####
# This script was imported from:
# https://answers.ros.org/question/230120/extracting-definition-of-customized-messages-from-bag-files/
# It can be used to extract message definitions of custom msgs from a ros bag file
# It needs the position/name of the bag file(s) as command line arguments
# Example:
# "rosrun T2TF_SST reconstructMsg data/bagName.bag"
#####

types = {}

for bagname in sys.argv[1:]:
    bag = rosbag.Bag(bagname)
    for topic, msg, t in bag.read_messages():
        if not msg._type in types:
            types[msg._type] = msg._full_text


for t in types:
    print("Message type:", t)
    print("Message text:")
    print(types[t])
    print("")
