import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
import pickle

import numpy as np
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from plot.helper import plot_joints, plot_task, plot_weights, plot_rf_z_max, plot_rf, plot_vector_traj

tasks = [
    'task_rfoot_pos', 'task_rfoot_vel', 'task_rfoot_ori', 'task_rfoot_ang_vel',
    'task_lfoot_pos', 'task_lfoot_vel', 'task_lfoot_ori', 'task_lfoot_ang_vel',
    'task_rhand_pos', 'task_rhand_vel', 'task_rhand_ori', 'task_rhand_ang_vel',
    'task_lhand_pos', 'task_lhand_vel', 'task_lhand_ori', 'task_lhand_ang_vel',
    'task_neck_pos', 'task_neck_vel'
]

neck_pos_label = ["neck_pitch"]

rfoot_label = [
    "r_hip_ie", "r_hip_aa", "r_hip_fe", "r_knee_fe_jp", "r_knee_fe_jd",
    "r_ankle_fe", "r_ankle_ie"
]

lfoot_label = [
    "l_hip_ie", "l_hip_aa", "l_hip_fe", "l_knee_fe_jp", "l_knee_fe_jd",
    "l_ankle_fe", "l_ankle_ie"
]

joint_label = [
    "l_hip_ie", "l_hip_aa", "l_hip_fe", "l_knee_fe_jp", "l_knee_fe_jd",
    "l_ankle_fe", "l_ankle_ie", "l_shoulder_fe", "l_shoulder_aa",
    "l_shoulder_ie", "l_elbow_fe", "l_wrist_ps", "l_wrist_pitch", "neck_pitch",
    "r_hip_ie", "r_hip_aa", "r_hip_fe", "r_knee_fe_jp", "r_knee_fe_jd",
    "r_ankle_fe", "r_ankle_ie", "r_shoulder_fe", "r_shoulder_aa",
    "r_shoulder_ie", "r_elbow_fe", "r_wrist_ps", "r_wrist_pitch"
]

time = []

phase = []

cmd_lfoot_rf = []
cmd_rfoot_rf = []

f_int = []

cmd_joint_positions = []
cmd_joint_velocities = []
cmd_joint_torques = []
joint_positions = []
joint_velocities = []

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
            cmd_joint_positions.append(d['cmd_joint_positions'])
            cmd_joint_velocities.append(d['cmd_joint_velocities'])
            cmd_joint_torques.append(d['cmd_joint_torques'])
            joint_positions.append(d['joint_positions'])
            joint_velocities.append(d['joint_velocities'])
            f_int.append(d['f_int'])
        except EOFError:
            break

for k, v in des.items():
    des[k] = np.stack(v, axis=0)
for k, v in act.items():
    act[k] = np.stack(v, axis=0)
# right foot first
phase = np.stack(phase, axis=0)
cmd_joint_positions = np.stack(cmd_joint_positions, axis=0)
cmd_joint_velocities = np.stack(cmd_joint_velocities, axis=0)
cmd_joint_torques = np.stack(cmd_joint_torques, axis=0)
joint_positions = np.stack(joint_positions, axis=0)
joint_velocities = np.stack(joint_velocities, axis=0)
f_int = np.stack(f_int, axis=0)

## =============================================================================
## Plot Task
## =============================================================================

# plot_task(time, des['task_lfoot_pos'], act['task_lfoot_pos'],
# des['task_lfoot_vel'], act['task_lfoot_vel'], phase, 'left foot lin')

# plot_task(time, des['task_lfoot_ori'], act['task_lfoot_ori'],
# des['task_lfoot_ang_vel'], act['task_lfoot_ang_vel'], phase,
# 'left foot ori')

# plot_task(time, des['task_rfoot_pos'], act['task_rfoot_pos'],
# des['task_rfoot_vel'], act['task_rfoot_vel'], phase,
# 'right foot lin')

# plot_task(time, des['task_rfoot_ori'], act['task_rfoot_ori'],
# des['task_rfoot_ang_vel'], act['task_rfoot_ang_vel'], phase,
# 'right foot ori')

# plot_task(time, des['task_lhand_pos'], act['task_lhand_pos'],
# des['task_lhand_vel'], act['task_lhand_vel'], phase, 'left hand lin')

# plot_task(time, des['task_lhand_ori'], act['task_lhand_ori'],
# des['task_lhand_ang_vel'], act['task_lhand_ang_vel'], phase,
# 'left hand ori')

# plot_task(time, des['task_rhand_pos'], act['task_rhand_pos'],
# des['task_rhand_vel'], act['task_rhand_vel'], phase,
# 'right hand lin')

# plot_task(time, des['task_rhand_ori'], act['task_rhand_ori'],
# des['task_rhand_ang_vel'], act['task_rhand_ang_vel'], phase,
# 'right hand ori')

## =============================================================================
## Plot WBC Solutions
## =============================================================================

plot_joints(joint_label, rfoot_label, time, cmd_joint_positions,
            joint_positions, cmd_joint_velocities, joint_velocities,
            cmd_joint_torques, phase, "rfoot")

plot_joints(joint_label, lfoot_label, time, cmd_joint_positions,
            joint_positions, cmd_joint_velocities, joint_velocities,
            cmd_joint_torques, phase, "lfoot")

l_knee_jd_idx = joint_label.index("l_knee_fe_jd")
r_knee_jd_idx = joint_label.index("r_knee_fe_jd")

fig, axes = plt.subplots(2, 1)
axes[0].set_title('left knee')
axes[1].set_title('right knee')
axes[0].grid(True)
axes[1].grid(True)
axes[0].plot(time, cmd_joint_torques[:, l_knee_jd_idx] / 2., 'r')
axes[0].plot(time, f_int[:, 0], 'b')
axes[1].plot(time, cmd_joint_torques[:, r_knee_jd_idx] / 2., 'r')
axes[1].plot(time, f_int[:, 1], 'b')

plt.show()
