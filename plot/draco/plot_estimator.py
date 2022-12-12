import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
import pickle

import numpy as np
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from plot.helper import *

from utils.python_utils.util import quat_to_rpy

st_idx = 10

time = []
phase = []
com_vel_est = []
com_vel_est_exp = []
com_vel_raw = []
imu_ang_vel_est = []
imu_ang_vel_raw = []
cam_est = []
cam_raw = []

base_joint_pos = []
base_joint_rpy = []
base_joint_lin_vel = []
base_joint_ang_vel = []

rf_sg = []
lf_sg = []

with open('experiment_data/pnc.pkl', 'rb') as file:
    while True:
        try:
            d = pickle.load(file)
            time.append(d['time'])
            phase.append(d['phase'])
            rf_sg.append(d['rf_sg'])
            lf_sg.append(d['lf_sg'])
            com_vel_est.append(d['com_vel_est'])
            com_vel_est_exp.append(d['com_vel_est_exp'])
            com_vel_raw.append(d['com_vel_raw'])
            imu_ang_vel_est.append(d['imu_ang_vel_est'])
            imu_ang_vel_raw.append(d['imu_ang_vel_raw'])
            cam_est.append(d['cam_est'])
            cam_raw.append(d['cam_raw'])
            base_joint_pos.append(d['base_joint_position'])
            base_joint_rpy.append(quat_to_rpy(d['base_joint_quat']))
            base_joint_lin_vel.append(d['base_joint_lin_vel'])
            base_joint_ang_vel.append(d['imu_ang_vel_raw'])

        except EOFError:
            break

time = np.array(time)[st_idx:]
phase = np.array(phase)[st_idx:]
rf_sg = np.array(rf_sg)[st_idx:]
lf_sg = np.array(lf_sg)[st_idx:]
com_vel_est = np.stack(com_vel_est, axis=0)[st_idx:, :]
com_vel_est_exp = np.stack(com_vel_est_exp, axis=0)[st_idx:, :]
com_vel_raw = np.stack(com_vel_raw, axis=0)[st_idx:, :]
imu_ang_vel_est = np.stack(imu_ang_vel_est, axis=0)[st_idx:, :]
imu_ang_vel_raw = np.stack(imu_ang_vel_raw, axis=0)[st_idx:, :]
cam_est = np.stack(cam_est, axis=0)[st_idx:, :]
cam_raw = np.stack(cam_raw, axis=0)[st_idx:, :]

base_joint_pos = np.stack(base_joint_pos, axis=0)[st_idx:, :]
base_joint_rpy = np.stack(base_joint_rpy, axis=0)[st_idx:, :]
base_joint_lin_vel = np.stack(base_joint_lin_vel, axis=0)[st_idx:, :]
base_joint_ang_vel = np.stack(base_joint_ang_vel, axis=0)[st_idx:, :]

axes = plot_vector_traj(time, com_vel_raw, phase, ["x", "y", "z"], "k",
                        "com vel")
axes = plot_vector_traj(time,
                        com_vel_est,
                        phase, ["x", "y", "z"],
                        "g",
                        axes=axes)

plot_vector_traj(time, com_vel_est_exp, phase, ["x", "y", "z"], "c", axes=axes)

axes = plot_vector_traj(time, imu_ang_vel_raw, phase, ["x", "y", "z"], "k",
                        "imu ang vel")
plot_vector_traj(time, imu_ang_vel_est, phase, ["x", "y", "z"], "g", axes=axes)
axes = plot_vector_traj(time, cam_raw, phase, ["x", "y", "z"], "k", "cam")
plot_vector_traj(time, cam_est, phase, ["x", "y", "z"], "g", axes=axes)

plot_vector_traj(time, base_joint_pos, phase, ["x", "y", "z"], "k",
                 "base_joint_pos")
plot_vector_traj(time, base_joint_rpy, phase, ["r", "p", "y"], "k",
                 "base_joint_rpy")
plot_vector_traj(time, base_joint_lin_vel, phase, ["x_dot", "y_dot", "z_dot"],
                 "k", "base_joint_lin_vel")
plot_vector_traj(time, base_joint_ang_vel, phase, ["w_x", "w_y", "w_z"], "k",
                 "base_joint_ang_vel")
axes = plot_scalar_traj(time, rf_sg, phase, "rf_sg", "k")
axes = plot_scalar_traj(time, lf_sg, phase, "lf_sg", "k")

plt.show()
