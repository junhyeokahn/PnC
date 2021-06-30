import os
import sys
cwd = os.getcwd()
sys.path.append(cwd)
import pickle

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from plot.helper import plot_task, plot_rf, plot_vector_traj

## Read Data

tasks_name = [
    'com_pos', 'com_vel', 'base_ori', 'base_angvel', 'rfoot_pos', 'rfoot_vel',
    'lfoot_pos', 'lfoot_vel', 'rfoot_ori', 'rfoot_angvel', 'lfoot_ori',
    'lfoot_angvel', 'sj_pos', 'sj_vel'
]

time = np.genfromtxt('experiment_data/time.txt', delimiter='\n', dtype=float)

des, act = dict(), dict()

for task_name in tasks_name:
    des[task_name] = np.genfromtxt('experiment_data/' + task_name + '_d.txt',
                                   delimiter=None,
                                   dtype=float)
    act[task_name] = np.genfromtxt('experiment_data/' + task_name + '.txt',
                                   delimiter=None,
                                   dtype=float)

phase = np.genfromtxt('experiment_data/phase.txt', delimiter='\n', dtype=int)

r_rf_cmd = np.genfromtxt('experiment_data/cmd_r_rf.txt',
                         delimiter=None,
                         dtype=float)

l_rf_cmd = np.genfromtxt('experiment_data/cmd_l_rf.txt',
                         delimiter=None,
                         dtype=float)

rf_cmd = np.concatenate((r_rf_cmd, l_rf_cmd), axis=1)

## Plot

plot_task(time, des['com_pos'], act['com_pos'], des['com_vel'], act['com_vel'],
          phase, 'com lin')

plot_task(time, des['base_ori'], act['base_ori'], des['base_angvel'],
          act['base_angvel'], phase, 'pelvis ori')

plot_task(time, des['rfoot_pos'], act['rfoot_pos'], des['rfoot_vel'],
          act['rfoot_vel'], phase, 'right foot lin')

plot_task(time, des['lfoot_pos'], act['lfoot_pos'], des['lfoot_vel'],
          act['lfoot_vel'], phase, 'left foot lin')

plot_task(time, des['rfoot_ori'], act['rfoot_ori'], des['rfoot_angvel'],
          act['rfoot_angvel'], phase, 'right foot ori')

plot_task(time, des['lfoot_ori'], act['lfoot_ori'], des['lfoot_angvel'],
          act['lfoot_angvel'], phase, 'left foot ori')

plot_task(time, des['sj_pos'], act['sj_pos'], des['sj_vel'], act['sj_vel'],
          phase, 'upper body joint')

plot_rf(time, rf_cmd, phase)

plt.show()
