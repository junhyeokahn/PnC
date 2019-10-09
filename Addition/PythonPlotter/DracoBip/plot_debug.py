import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

figure_number = 0
col_index = 0
row_index = 0

file_path = os.getcwd() + "/../../../ExperimentData/"

## read files
delta_q = []
delta_x = []
for i in range(4):
    delta_q.append(np.genfromtxt(file_path+'delta_q'+str(i)+'.txt', delimiter=None, dtype=(float)))
    delta_x.append(np.genfromtxt(file_path+'delta_x'+str(i)+'.txt', delimiter=None, dtype=(float)))

axes = plt.gca()

fig = plt.figure(figure_number)
fig.canvas.set_window_title('com pos')

st_idx = 1
end_idx = len(delta_x[0]) - 10

n_task = len(delta_q)
for task_idx in range(n_task):
    fig = plt.figure(figure_number)
    fig.canvas.set_window_title('delta_x'+str(task_idx))
    n_dim = delta_x[task_idx].shape[1]
    for dim_idx in range(n_dim):
        ax1 = plt.subplot(n_dim, 1, dim_idx+1)
        plt.plot(delta_x[task_idx][st_idx:end_idx, dim_idx], 'b-')
        plt.grid(True)
    plt.xlabel('idx')
    figure_number += 1

    fig = plt.figure(figure_number)
    fig.canvas.set_window_title('delta_q'+str(task_idx))
    n_dim = delta_q[task_idx].shape[1]
    for dim_idx in range(n_dim):
        ax1 = plt.subplot(int(np.ceil(n_dim/2)), 2, dim_idx+1)
        plt.plot(delta_q[task_idx][st_idx:end_idx, dim_idx], 'r-')
        plt.grid(True)
    plt.xlabel('idx')
    figure_number += 1


plt.show()
