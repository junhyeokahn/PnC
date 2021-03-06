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
    'task_lfoot_pos', 'task_lfoot_vel', 'task_lfoot_ori', 'task_lfoot_ang_vel',
    'task_upper_body_pos', 'task_upper_body_vel'
]

upper_body_pos_label = [
"neck_pitch", "l_shoulder_fe", "l_shoulder_aa", "l_shoulder_ie",
"l_elbow_fe", "l_wrist_ps", "l_wrist_pitch", "r_shoulder_fe",
"r_shoulder_aa", "r_shoulder_ie", "r_elbow_fe", "r_wrist_ps",
"r_wrist_pitch"
]

joint_label = [
"l_hip_ie", "l_hip_aa", "l_hip_fe", "l_knee_fe_jp", "l_knee_fe_jd",
"l_ankle_fe", "l_ankle_ie", "l_shoulder_fe", "l_shoulder_aa", "l_shoulder_ie",
"l_elbow_fe", "l_wrist_ps", "l_wrist_pitch", "neck_pitch", "r_hip_ie", "r_hip_aa",
"r_hip_fe", "r_knee_fe_jp", "r_knee_fe_jd", "r_ankle_fe", "r_ankle_ie",
"r_shoulder_fe", "r_shoulder_aa", "r_shoulder_ie", "r_elbow_fe", "r_wrist_ps", "r_wrist_pitch"
]

time = []

phase = []

cmd_lfoot_rf = []
cmd_rfoot_rf = []

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
            cmd_lfoot_rf.append(d['cmd_lfoot_rf'])
            cmd_rfoot_rf.append(d['cmd_rfoot_rf'])
            cmd_joint_positions.append(d['cmd_joint_positions'])
            cmd_joint_velocities.append(d['cmd_joint_velocities'])
            cmd_joint_torques.append(d['cmd_joint_torques'])
            joint_positions.append(d['joint_positions'])
            joint_velocities.append(d['joint_velocities'])
        except EOFError:
            break

for k, v in des.items():
    des[k] = np.stack(v, axis=0)
for k, v in act.items():
    act[k] = np.stack(v, axis=0)
# right foot first
cmd_rf = np.concatenate((cmd_rfoot_rf, cmd_lfoot_rf), axis=1)
phase = np.stack(phase, axis=0)
cmd_joint_positions=np.stack(cmd_joint_positions, axis=0)
cmd_joint_velocities=np.stack(cmd_joint_velocities, axis=0)
cmd_joint_torques=np.stack(cmd_joint_torques, axis=0)
joint_positions=np.stack(joint_positions, axis=0)
joint_velocities=np.stack(joint_velocities, axis=0)

## =============================================================================
## Plot Task
## =============================================================================

plot_task(time, des['task_com_pos'], act['task_com_pos'], des['task_com_vel'],
          act['task_com_vel'], phase, 'com lin')

plot_task(time, des['task_torso_ori'], act['task_torso_ori'],
          des['task_torso_ang_vel'], act['task_torso_ang_vel'], phase,
          'torso ori')

for i in range(3):
    slc = slice(5*i, 5*(i+1))
    plot_task(time, des['task_upper_body_pos'][:,slc], act['task_upper_body_pos'][:,slc],
          des['task_upper_body_vel'][:,slc], act['task_upper_body_vel'][:,slc], phase,
          'upper body', upper_body_pos_label[slc])

# for i in range(6):
    # slc = slice(5*i, 5*(i+1))
    # plot_vector_traj(time, cmd_joint_torques[:,slc], phase,
        # 'joint torque command', joint_label[slc])

# for i in range(6):
    # slc = slice(5*i, 5*(i+1))
    # plot_task(time, cmd_joint_positions[:,slc], joint_positions[:,slc],
        # cmd_joint_velocities[:,slc], joint_velocities[:,slc], phase,
        # 'joint', joint_label[slc])

# plot_task(time, cmd_joint_positions[:,5:10], joint_positions[:,5:10],
        # cmd_joint_velocities[:,5:10], joint_velocities[:,5:10], phase,
        # 'joint', joint_label[5:10])

# plot_task(time, cmd_joint_positions[:,10:15], joint_positions[:,10:15],
        # cmd_joint_velocities[:,10:15], joint_velocities[:,10:15], phase,
        # 'joint', joint_label[10:15])

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

## =============================================================================
## Plot WBC Solutions
## =============================================================================
plot_rf(time, cmd_rf, phase)  # assume top six is right foot

plt.show()
