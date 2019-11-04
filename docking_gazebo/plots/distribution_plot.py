#!usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import Axes3D
import sys
import numpy as np
import seaborn as sns
import scipy.stats as stats

sns.set(color_codes=True)

directory = "/home/ryrocha/catkin_ws/src/docking_simulator/docking_gazebo/plots/"

filtered_df = pd.read_csv(directory + str(sys.argv[1]), header=None, names=['x', 'y', 'z'])

df_rows = filtered_df.shape[0]


plt.figure()
plt.plot(filtered_df.index, filtered_df['x'], label='position')
plt.plot(filtered_df.index, np.full(df_rows, 0.1075), c='k', lw=1, ls= '--', label='desired limit')
plt.plot(filtered_df.index, np.full(df_rows, -0.1075), c='k', lw=1, ls= '--')
plt.ylabel('x [m]')
plt.grid(True)
plt.legend()

plt.figure()
plt.plot(filtered_df.index, filtered_df['y'], label='position')
plt.plot(filtered_df.index, np.full(df_rows, 0.1075), c='k', lw=1, ls= '--', label='desired limit')
plt.plot(filtered_df.index, np.full(df_rows, -0.1075), c='k', lw=1, ls= '--')
plt.ylabel('y [m]')
plt.grid(True)
plt.legend()


fig = plt.figure(figsize=(8, 8))
grid = plt.GridSpec(4, 4, hspace=0.05, wspace=0.05)
main_ax = fig.add_subplot(grid[1:, :3])
x_hist = fig.add_subplot(grid[0, :3], yticklabels=[], sharex=main_ax)
y_hist = fig.add_subplot(grid[1:, 3], xticklabels=[], sharey=main_ax)

x_data = filtered_df['x'][450:]
y_data = filtered_df['y'][450:]

main_ax.scatter(x_data, y_data, edgecolors='none')
circle1=plt.Circle((0, 0), 0.1075, color='k', ls='--', lw='1.5', fill=False)
main_ax.add_artist(circle1)
legend_elements = [Line2D([0], [0], marker='o', color='b', label='position',
					      markerfacecolor='b', markersize=5, linestyle=''),
                   Line2D([0], [0], marker=u'$\u25CC$', color='k', label=' desired limit', 
                   		  markerfacecolor='none', markersize=15, linestyle='')]
main_ax.set_xlim([-0.2, 0.2])
main_ax.set_ylim([-0.2, 0.2])
main_ax.set_xlabel('x [m]')
main_ax.set_ylabel('y [m]')
main_ax.legend(handles=legend_elements)

xt = main_ax.get_xticks()
xmin, xmax = min(xt), max(xt) 
lnspc_x = np.linspace(xmin, xmax, df_rows)
m_x, s_x = stats.norm.fit(x_data) 
pdf_x = stats.norm.pdf(lnspc_x, m_x, s_x)  
x_hist.plot(lnspc_x, pdf_x, label="Norm")
x_hist.hist(x_data, bins=15, color='b', alpha = 0.5, density=True)
x_hist.xaxis.set_visible(False)

yt = main_ax.get_yticks()
ymin, ymax = min(yt), max(yt) 
lnspc_y = np.linspace(ymin, ymax, df_rows)
m_y, s_y = stats.norm.fit(y_data) 
pdf_y = stats.norm.pdf(lnspc_y, m_y, s_y)  
y_hist.plot(pdf_y, lnspc_y, label="Norm")
y_hist.hist(y_data, bins=15, color='b', alpha = 0.5, orientation='horizontal', density=True)
y_hist.yaxis.set_visible(False)


axmin3D = -0.75
axmax3D = 0.75
lnspc3D = np.linspace(axmin3D, axmax3D, df_rows)

X, Y = np.meshgrid(lnspc3D, lnspc3D)

pos = np.empty(X.shape + (2,))
pos[:, :, 0] = X; pos[:, :, 1] = Y
rv = stats.multivariate_normal([m_x, m_y], [[s_x, 0], [0, s_y]])

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot_surface(X, Y, rv.pdf(pos),cmap='viridis',linewidth=0)
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')


plt.show()