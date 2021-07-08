import os
import sys
cwd = os.getcwd()
sys.path.append(cwd)
import pickle

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from plot.helper import plot_task, plot_weights, plot_rf_z_max, plot_rf, plot_vector_traj

tasks = [
    'task_com_pos', 'task_com_vel', 'task_torso_ori', 'task_torso_ang_vel',
    'task_rfoot_pos', 'task_rfoot_vel', 'task_rfoot_ori', 'task_rfoot_ang_vel',
    'task_lfoot_pos', 'task_lfoot_vel', 'task_lfoot_ori', 'task_lfoot_ang_vel'
]

time = []

phase = []

cmd_lfoot_rf = []
cmd_rfoot_rf = []

des, act = dict(), dict()
for topic in tasks:
    des[topic] = []
    act[topic] = []
w = dict()

with open('experiment_data/pnc.pkl', 'rb') as file:
    while True:
        try:
            d = pickle.load(file)
            time.append(d['time'])
            phase.append(d['phase'])
            for topic in tasks:
                des[topic].append(d[topic + '_des'])
                act[topic].append(d[topic])
            cmd_lfoot_rf.append(d['cmd_lfoot_rf'])
            cmd_rfoot_rf.append(d['cmd_rfoot_rf'])
        except EOFError:
            break

for k, v in des.items():
    des[k] = np.stack(v, axis=0)
for k, v in act.items():
    act[k] = np.stack(v, axis=0)
# right foot first
cmd_rf = np.concatenate((cmd_rfoot_rf, cmd_lfoot_rf), axis=1)
phase = np.stack(phase, axis=0)

## =============================================================================
## Plot Task
## =============================================================================

plot_task(time, des['task_com_pos'], act['task_com_pos'], des['task_com_vel'],
          act['task_com_vel'], phase, 'com lin')

plot_task(time, des['task_torso_ori'], act['task_torso_ori'],
          des['task_torso_ang_vel'], act['task_torso_ang_vel'], phase,
          'torso ori')

plot_task(time, des['task_lfoot_pos'], act['task_lfoot_pos'],
          des['task_lfoot_vel'], act['task_lfoot_vel'], phase, 'left foot lin')

plot_task(time, des['task_lfoot_ori'], act['task_lfoot_ori'],
          des['task_lfoot_ang_vel'], act['task_lfoot_ang_vel'], phase,
          'left foot ori')

plot_task(time, des['task_rfoot_pos'], act['task_rfoot_pos'],
          des['task_rfoot_vel'], act['task_rfoot_vel'], phase,
          'right foot lin')

plot_task(time, des['task_rfoot_ori'], act['task_rfoot_ori'],
          des['task_rfoot_ang_vel'], act['task_rfoot_ang_vel'], phase,
          'right foot ori')

## =============================================================================
## Plot WBC Solutions
## =============================================================================
plot_rf(time, cmd_rf, phase)  # assume top six is right foot

plt.show()
