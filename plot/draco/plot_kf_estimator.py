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

from utils.python_utils.util import quat_to_rpy

st_idx = 10

time = []
phase = []
com_vel_est = []
com_vel_raw = []
imu_ang_vel_est = []
imu_ang_vel_raw = []
imu_acceleration = []
cam_est = []
cam_raw = []

base_joint_pos = []
base_joint_rpy = []
base_joint_lin_vel = []
base_joint_ang_vel = []

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
            base_joint_pos.append(d['base_pos_kf'])
            base_joint_rpy.append(quat_to_rpy(d['base_quat_kf']))
            base_joint_lin_vel.append(d['base_vel_kf'])
            base_joint_ang_vel.append(d['imu_ang_vel_raw'])
            imu_acceleration.append(d['imu_accel'])

        except EOFError:
            break

time = np.array(time)[st_idx:]
phase = np.array(phase)[st_idx:]
imu_acceleration = np.stack(imu_acceleration, axis=0)[st_idx:, :]
com_vel_est = np.stack(com_vel_est, axis=0)[st_idx:, :]
com_vel_raw = np.stack(com_vel_raw, axis=0)[st_idx:, :]
imu_ang_vel_est = np.stack(imu_ang_vel_est, axis=0)[st_idx:, :]
imu_ang_vel_raw = np.stack(imu_ang_vel_raw, axis=0)[st_idx:, :]
cam_est = np.stack(cam_est, axis=0)[st_idx:, :]
cam_raw = np.stack(cam_raw, axis=0)[st_idx:, :]

base_joint_pos = np.stack(base_joint_pos, axis=0)[st_idx:, :]
base_joint_rpy = np.stack(base_joint_rpy, axis=0)[st_idx:, :]
base_joint_lin_vel = np.stack(base_joint_lin_vel, axis=0)[st_idx:, :]
base_joint_ang_vel = np.stack(base_joint_ang_vel, axis=0)[st_idx:, :]

plot_vector_traj(time, imu_acceleration, phase, ["x", "y", "z"], "g",
                 "imu_acceleration")

axes = plot_vector_traj(time, com_vel_raw, phase, ["x", "y", "z"], "k",
                        "com vel")
plot_vector_traj(time, com_vel_est, phase, ["x", "y", "z"], "g", None, axes)

axes = plot_vector_traj(time, imu_ang_vel_raw, phase, ["x", "y", "z"], "k",
                        "imu ang vel")
plot_vector_traj(time, imu_ang_vel_est, phase, ["x", "y", "z"], "g", None,
                 axes)
axes = plot_vector_traj(time, cam_raw, phase, ["x", "y", "z"], "k", "cam")
plot_vector_traj(time, cam_est, phase, ["x", "y", "z"], "g", None, axes)

plot_vector_traj(time, base_joint_pos, phase, ["x", "y", "z"], "k",
                 "base_joint_pos")
plot_vector_traj(time, base_joint_rpy, phase, ["r", "p", "y"], "k",
                 "base_joint_rpy")
plot_vector_traj(time, base_joint_lin_vel, phase, ["x_dot", "y_dot", "z_dot"],
                 "k", "base_joint_lin_vel")
plot_vector_traj(time, base_joint_ang_vel, phase, ["w_x", "w_y", "w_z"], "k",
                 "base_joint_ang_vel")

plt.show()
