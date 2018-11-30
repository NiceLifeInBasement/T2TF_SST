#!/usr/bin/env python
"""
Used to profile the statistic of other nodes

assoc_long.txt holds a profile for historic_assoc with Duration(5)
assoc_short.txt holds a profile for historic_assoc with Duration(0)
"""
import cProfile
from historic_assoc import *
import os
import rospy
import pstats

filename = "hist_stats_2.txt"

# --------

# The following block uses .Profile() to generate a profile that is dumped into the file
# pr = cProfile.Profile()
# pr.enable()
# import historic_assoc
# historic_assoc.setup(['/home/simon/catkin_ws/src/T2TF_SST/src/historic_assoc.py', 'maven-1.bag'])
# pr.disable()
# pr.create_stats()
# pr.dump_stats(filename)

# Use the following line if you want to re-generate the data
# cProfile.run("historic_assoc.setup(['/home/simon/catkin_ws/src/T2TF_SST/src/historic_assoc.py', 'maven-1.bag'])", filename)

# IMPORT THE DATA FROM THE FILE
p = pstats.Stats("assoc_long.txt")
# the following is just analysis of data:
# p.strip_dirs().sort_stats(-1).print_stats()


p.sort_stats('tottime').print_stats()


