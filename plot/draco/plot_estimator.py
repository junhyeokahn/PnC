import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
import pickle

import numpy as np
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from plot.helper import plot_vector_traj

st_idx = 10

time = []
phase = []
com_vel_est = []
com_vel_raw = []
imu_ang_vel_est = []
imu_ang_vel_raw = []
cam_est = []
cam_raw = []

with open('experiment_data/pnc.pkl', 'rb') as file:
    while True:
        try:
            d = pickle.load(file)
            time.append(d['time'])
            phase.append(d['phase'])
            com_vel_est.append(d['com_vel_est'])
            com_vel_raw.append(d['com_vel_raw'])
            imu_ang_vel_est.append(d['imu_ang_vel_est'])
            imu_ang_vel_raw.append(d['imu_ang_vel_raw'])
            cam_est.append(d['cam_est'])
            cam_raw.append(d['cam_raw'])
        except EOFError:
            break

time = np.array(time)[st_idx:]
phase = np.array(phase)[st_idx:]
com_vel_est = np.stack(com_vel_est, axis=0)[st_idx:, :]
com_vel_raw = np.stack(com_vel_raw, axis=0)[st_idx:, :]
imu_ang_vel_est = np.stack(imu_ang_vel_est, axis=0)[st_idx:, :]
imu_ang_vel_raw = np.stack(imu_ang_vel_raw, axis=0)[st_idx:, :]
cam_est = np.stack(cam_est, axis=0)[st_idx:, :]
cam_raw = np.stack(cam_raw, axis=0)[st_idx:, :]

axes = plot_vector_traj(time, com_vel_raw, phase, ["x", "y", "z"], "k",
                        "com vel")
plot_vector_traj(time, com_vel_est, phase, ["x", "y", "z"], "g", None, axes)

axes = plot_vector_traj(time, imu_ang_vel_raw, phase, ["x", "y", "z"], "k",
                        "imu ang vel")
plot_vector_traj(time, imu_ang_vel_est, phase, ["x", "y", "z"], "g", None,
                 axes)
axes = plot_vector_traj(time, cam_raw, phase, ["x", "y", "z"], "k", "cam")
plot_vector_traj(time, cam_est, phase, ["x", "y", "z"], "g", None, axes)

plt.show()
