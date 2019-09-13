#!usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import sys

directory = "/home/ryrocha/catkin_ws/src/docking_simulator/docking_gazebo/plots/"

raw_df = pd.read_csv(directory + str(sys.argv[1]), header=None, names=['x', 'y', 'z'])
filtered_df = pd.read_csv(directory + str(sys.argv[2]), header=None, names=['x', 'y', 'z'])
# setpoint_df = pd.read_csv(directory + str(sys.argv[3]), header=None, names=['x', 'y', 'z'])

plt.figure()
plt.scatter(raw_df.index, raw_df['x'], s=1.0, c='k')
plt.plot(filtered_df.index, filtered_df['x'], 'g')
# plt.plot(setpoint_df.index, setpoint_df['x'], 'b')

plt.figure()
plt.scatter(raw_df.index, raw_df['y'], s=1.0, c='k')
plt.plot(filtered_df.index, filtered_df['y'], 'g')
# plt.plot(setpoint_df.index, setpoint_df['x'], 'b')

plt.show()