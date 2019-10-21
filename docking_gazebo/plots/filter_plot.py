#!usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import sys

directory = "/home/ryrocha/catkin_ws/src/docking_simulator/docking_gazebo/plots/"

# raw_df = pd.read_csv(directory + str(sys.argv[1]), header=None, names=['x', 'y', 'z'])
filtered_df = pd.read_csv(directory + str(sys.argv[1]), header=None, names=['x', 'y', 'z'])
current_df = pd.read_csv(directory + str(sys.argv[2]), header=None, names=['x', 'y', 'z'])

plt.figure()
# plt.plot(raw_df.index, raw_df['x'], 'k', label='raw cv')
plt.plot(filtered_df.index, filtered_df['x'] - 1.53259194, 'g', label='transform cv')
plt.plot(current_df.index, current_df['x'], 'b', lw=1, label='px4')
plt.xlabel('iteration')
plt.ylabel('x [m]')
plt.grid(True)
plt.legend()

plt.figure()
# plt.plot(raw_df.index, raw_df['y'], 'k', label='raw cv')
plt.plot(filtered_df.index, filtered_df['y'] - 2.3844518, 'g', label='transform cv')
plt.plot(current_df.index, current_df['y'], 'b', label='px4')
plt.xlabel('iteration')
plt.ylabel('y [m]')
plt.grid(True)
plt.legend()

plt.figure()
# plt.plot(raw_df.index, raw_df['z'], 'k', label='raw cv')
plt.plot(filtered_df.index, filtered_df['z'], 'g', label='transform cv')
plt.plot(current_df.index, current_df['z'], 'b', label='px4')
plt.xlabel('iteration')
plt.ylabel('z [m]')
plt.grid(True)
plt.legend()


plt.show()