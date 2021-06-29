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
    'com_pos', 'com_vel', 'pelvis_com_ori', 'pelvis_com_angvel', 'r_sole_pos',
    'r_sole_vel', 'l_sole_pos', 'l_sole_vel', 'r_sole_ori', 'r_sole_ori',
    'r_sole_angvel', 'l_sole_ori', 'l_sole_angvel', 'sj_pos', 'sj_vel'
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

plot_task(time, des['pelvis_com_ori'], act['pelvis_com_ori'],
          des['pelvis_com_angvel'], act['pelvis_com_angvel'], phase,
          'pelvis ori')

plot_task(time, des['r_sole_pos'], act['r_sole_pos'], des['r_sole_vel'],
          act['r_sole_vel'], phase, 'right foot lin')

plot_task(time, des['l_sole_pos'], act['l_sole_pos'], des['l_sole_vel'],
          act['l_sole_vel'], phase, 'left foot lin')

plot_task(time, des['r_sole_ori'], act['r_sole_ori'], des['r_sole_angvel'],
          act['r_sole_angvel'], phase, 'right foot ori')

plot_task(time, des['l_sole_ori'], act['l_sole_ori'], des['l_sole_angvel'],
          act['l_sole_angvel'], phase, 'left foot ori')

plot_task(time, des['sj_pos'], act['sj_pos'], des['sj_vel'], act['sj_vel'],
          phase, 'upper body joint')

plot_rf(time, rf_cmd, phase)

plt.show()
