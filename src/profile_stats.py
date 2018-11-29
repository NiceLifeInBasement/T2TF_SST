#!/usr/bin/env python
"""
Used to profile the statistic of other nodes
"""
import cProfile
from historic_assoc import *
import os
import rospy
import historic_assoc
import pstats

filename = "hist_stats.txt"

# --------
# Use the following line if you want to re-generate the data
# cProfile.run("historic_assoc.setup(['/home/simon/catkin_ws/src/T2TF_SST/src/historic_assoc.py', 'maven-1.bag'])", filename)

# the following is just analysis of data:
p = pstats.Stats(filename)
#p.strip_dirs().sort_stats(-1).print_stats()

p.sort_stats('filename', 'cumulative').print_stats()

