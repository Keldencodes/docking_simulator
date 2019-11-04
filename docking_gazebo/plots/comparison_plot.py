#!usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import sys
import numpy as np
import seaborn as sns
import scipy.stats as stats

sns.set(color_codes=True)

directory = "/home/ryrocha/catkin_ws/src/docking_simulator/docking_gazebo/plots/"

df1 = pd.read_csv(directory + str(sys.argv[1]), header=None, names=['x', 'y', 'z'])
df2 = pd.read_csv(directory + str(sys.argv[2]), header=None, names=['x', 'y', 'z'])
df3 = pd.read_csv(directory + str(sys.argv[3]), header=None, names=['x', 'y', 'z'])
df4 = pd.read_csv(directory + str(sys.argv[4]), header=None, names=['x', 'y', 'z'])
# df5 = pd.read_csv(directory + str(sys.argv[5]), header=None, names=['x', 'y', 'z'])

df_rows = df1.shape[0]
axmin = -0.2
axmax = 0.2
lnspc = np.linspace(axmin, axmax, df_rows)

x_df1 = df1['x'][400:]
mx_df1, sx_df1 = stats.norm.fit(x_df1) 
pdfx_df1 = stats.norm.pdf(lnspc, mx_df1, sx_df1)
y_df1 = df1['y'][400:]
my_df1, sy_df1 = stats.norm.fit(y_df1) 
pdfy_df1 = stats.norm.pdf(lnspc, my_df1, sy_df1)

x_df2 = df2['x'][400:]
mx_df2, sx_df2 = stats.norm.fit(x_df2) 
pdfx_df2 = stats.norm.pdf(lnspc, mx_df2, sx_df2)
y_df2 = df2['y'][400:]
my_df2, sy_df2 = stats.norm.fit(y_df2) 
pdfy_df2 = stats.norm.pdf(lnspc, my_df2, sy_df2)

x_df3 = df3['x'][400:]
mx_df3, sx_df3 = stats.norm.fit(x_df3) 
pdfx_df3 = stats.norm.pdf(lnspc, mx_df3, sx_df3)
y_df3 = df3['y'][400:]
my_df3, sy_df3 = stats.norm.fit(y_df3) 
pdfy_df3 = stats.norm.pdf(lnspc, my_df3, sy_df3)

x_df4 = df4['x'][400:]
mx_df4, sx_df4 = stats.norm.fit(x_df4) 
pdfx_df4 = stats.norm.pdf(lnspc, mx_df4, sx_df4)
y_df4 = df4['y'][400:]
my_df4, sy_df4 = stats.norm.fit(y_df4) 
pdfy_df4 = stats.norm.pdf(lnspc, my_df4, sy_df4)

# x_df5 = df5['x'][400:]
# mx_df5, sx_df5 = stats.norm.fit(x_df5) 
# pdfx_df5 = stats.norm.pdf(lnspc, mx_df5, sx_df5)
# y_df5 = df5['y'][400:]
# my_df5, sy_df5 = stats.norm.fit(y_df5) 
# pdfy_df5 = stats.norm.pdf(lnspc, my_df5, sy_df5)

plt.figure()
ax1 = plt.subplot(211)
plt.plot(lnspc, pdfx_df1, label="0.05")
plt.plot(lnspc, pdfx_df2, label="0.06")
plt.plot(lnspc, pdfx_df3, label="0.07")
plt.plot(lnspc, pdfx_df4, label="0.08")
ax1.set_yticklabels([])
plt.legend()

ax2 = plt.subplot(212)
plt.plot(lnspc, pdfy_df1, label="0.05")
plt.plot(lnspc, pdfy_df2, label="0.06")
plt.plot(lnspc, pdfy_df3, label="0.07")
plt.plot(lnspc, pdfy_df4, label="0.08")
ax2.set_yticklabels([])

plt.show()