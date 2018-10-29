#!/usr/bin/env python
"""
File that provides analysis for all kinds of data related to the tracking consistency algorithm (and similarity)

Not a main file, more for quick scripting and testing
"""
import numpy as np
import rospy
import matplotlib.pyplot as plt
import copy
import sys

file_name = "src/T2TF_SST/data/"

if len(sys.argv) < 2:
    file_name += "maven1_A.txt"
else:
    file_name += sys.argv[1]

with open(file_name) as f:
    data = f.readlines()
data = [float(x.strip()) for x in data]

sorted_data = copy.copy(data)
sorted_data.sort()

plt.plot(sorted_data)
plt.ylabel('similarity values from maven-1.bag')
no_zero = 0
for i in range(0, len(sorted_data)):
    if sorted_data[i] == 0:
        no_zero += 1
    else:
        break
print("Number of Values (total):"+str(len(data)))
print("Average of Values: "+str(np.average(data)))
print("Number of 0 Values: "+str(no_zero))

plt.show()
