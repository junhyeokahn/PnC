import random

import numpy as np
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

xyz_label = ['x', 'y', 'z']
quat_label = ['w', 'x', ' y', ' z']
markers = ['*', '+', 'h', 'x', 'o', 'v', 'd']
# facecolors = [
# 'lightgray', 'rosybrown', 'cornflowerblue', 'darkorange', 'seagreen',
# 'gold', 'lightpink', 'slategrey'
# ]

facecolors = 3 * [
    'gray', 'brown', 'cornflowerblue', 'red', 'blue', 'yellow', 'purple',
    'crimson', 'red'
]


def plot_momentum_task(time, vel_des, vel, phase, suptitle, label=None):
    fig, axes = plt.subplots(3, 1)
    for i in range(3):
        axes[i].plot(time,
                     vel_des[:, i],
                     color='r',
                     linestyle='dashed',
                     linewidth=4)
        axes[i].plot(time, vel[:, i], color='b', linewidth=2)
        axes[i].grid(True)
        axes[i].set_ylabel(xyz_label[i] + 'dot')
        plot_phase(axes[i], time, phase)
    axes[2].set_xlabel('time')
    axes[2].set_xlabel('time')

    fig.suptitle(suptitle)


def plot_task(time, pos_des, pos, vel_des, vel, phase, suptitle, label=None):

    if pos_des.shape[1] == 3 and label is None:
        fig, axes = plt.subplots(3, 2)
        for i in range(3):
            axes[i, 0].plot(time,
                            pos_des[:, i],
                            color='r',
                            linestyle='dashed',
                            linewidth=4)
            axes[i, 0].plot(time, pos[:, i], color='b', linewidth=2)
            axes[i, 0].grid(True)
            axes[i, 0].set_ylabel(xyz_label[i])
            plot_phase(axes[i, 0], time, phase)
            axes[i, 1].plot(time,
                            vel_des[:, i],
                            color='r',
                            linestyle='dashed',
                            linewidth=4)
            axes[i, 1].plot(time, vel[:, i], color='b', linewidth=2)
            axes[i, 1].grid(True)
            axes[i, 1].set_ylabel(xyz_label[i] + 'dot')
            plot_phase(axes[i, 1], time, phase)
        axes[2, 0].set_xlabel('time')
        axes[2, 1].set_xlabel('time')
        fig.suptitle(suptitle)

    elif pos_des.shape[1] == 4 and label is None:
        fig, axes = plt.subplots(4, 2)
        for i in range(4):
            axes[i, 0].plot(time,
                            pos_des[:, i],
                            color='r',
                            linestyle='dashed',
                            linewidth=4)
            axes[i, 0].plot(time, pos[:, i], color='b', linewidth=2)
            axes[i, 0].grid(True)
            axes[i, 0].set_ylabel(quat_label[i])
            plot_phase(axes[i, 0], time, phase)
        for i in range(3):
            axes[i, 1].plot(time,
                            vel_des[:, i],
                            color='r',
                            linestyle='dashed',
                            linewidth=4)
            axes[i, 1].plot(time, vel[:, i], color='b', linewidth=2)
            plot_phase(axes[i, 1], time, phase)
            axes[i, 1].grid(True)
            axes[i, 1].set_ylabel(xyz_label[i] + 'dot')
        axes[3, 0].set_xlabel('time')
        axes[3, 1].set_xlabel('time')
        fig.suptitle(suptitle)

    else:
        dim = pos_des.shape[1]
        fig, axes = plt.subplots(dim, 2)
        for i in range(dim):
            axes[i, 0].plot(time,
                            pos_des[:, i],
                            color='r',
                            linestyle='dashed',
                            linewidth=4)
            axes[i, 0].plot(time, pos[:, i], color='b', linewidth=2)
            plot_phase(axes[i, 0], time, phase)
            axes[i, 0].grid(True)
            if label is not None:
                axes[i, 0].set_ylabel(label[i])
            axes[i, 1].plot(time,
                            vel_des[:, i],
                            color='r',
                            linestyle='dashed',
                            linewidth=4)
            axes[i, 1].plot(time, vel[:, i], color='b', linewidth=2)
            plot_phase(axes[i, 1], time, phase)
            axes[i, 1].grid(True)
        axes[dim - 1, 0].set_xlabel('time')
        axes[dim - 1, 1].set_xlabel('time')
        fig.suptitle(suptitle)


def plot_weights(time, weights_dict, phase):
    fig, ax = plt.subplots()
    for i, (k, v) in enumerate(weights_dict.items()):
        ax.plot(time,
                v,
                label=k,
                marker=markers[i],
                markersize=10,
                markevery=random.randint(50, 150))
    plot_phase(ax, time, phase)
    ax.grid(True)
    ax.set_xlabel('time')
    ax.legend()
    fig.suptitle('task weights')


def plot_rf_z_max(time, rf_z_max, phase):
    fig, ax = plt.subplots()
    for i, (k, v) in enumerate(rf_z_max.items()):
        ax.plot(time,
                v,
                label=k,
                marker=markers[i],
                markersize=10,
                markevery=random.randint(50, 150))
    plot_phase(ax, time, phase)
    ax.grid(True)
    ax.set_xlabel('time')
    ax.legend()
    fig.suptitle('rf_z_max')


def plot_vector_traj(time,
                     vector,
                     phase,
                     label,
                     color,
                     suptitle=None,
                     axes=None):
    dim = vector.shape[1]
    if axes is None:
        fig, axes = plt.subplots(dim, 1)
    for i in range(dim):
        axes[i].plot(time, vector[:, i], color=color, linewidth=3)
        axes[i].grid(True)
        if label is not None:
            axes[i].set_ylabel(label[i])
        if phase is not None:
            plot_phase(axes[i], time, phase)
    axes[dim - 1].set_xlabel('time')
    if (suptitle is not None):
        fig.suptitle(suptitle)
    return axes


def plot_rf(time, rfs, phase):
    fig, axes = plt.subplots(6, 2)
    for i in range(6):
        axes[i, 0].plot(time, rfs[:, i], color='k', linewidth=3)
        axes[i, 1].plot(time, rfs[:, i + 6], color='k', linewidth=3)
        axes[i, 0].grid(True)
        axes[i, 1].grid(True)
        plot_phase(axes[i, 0], time, phase)
        plot_phase(axes[i, 1], time, phase)
    axes[5, 0].set_xlabel('time')
    axes[5, 1].set_xlabel('time')
    axes[0, 0].set_title('Right Foot')
    axes[0, 1].set_title('Left Foot')
    fig.suptitle('Reaction Force Command')


def plot_phase(ax, t, data_phse):
    phseChange = []
    for i in range(0, len(t) - 1):
        if data_phse[i] != data_phse[i + 1]:
            phseChange.append(i)
        else:
            pass

    shading = 0.2
    prev_j = 0
    ll, ul = (ax.get_ylim())
    for j in (phseChange):
        ax.fill_between(t[prev_j:j + 1],
                        ll,
                        ul,
                        facecolor=facecolors[data_phse[j]],
                        alpha=shading)
        prev_j = j
    ax.fill_between(t[prev_j:],
                    ll,
                    ul,
                    facecolor=facecolors[data_phse[prev_j + 1]],
                    alpha=shading)


def plot_joints(full_joint, selected_joint, time, cmd_joint_positions,
                joint_positions, cmd_joint_velocities, joint_velocities,
                cmd_joint_torques, phase, title):
    idx_list = []
    for name in selected_joint:
        idx_list.append(full_joint.index(name))
    plot_task(time, cmd_joint_positions[:, idx_list],
              joint_positions[:, idx_list], cmd_joint_velocities[:, idx_list],
              joint_velocities[:, idx_list], phase, title, selected_joint)
    plot_vector_traj(time, cmd_joint_torques[:, idx_list], phase,
                     selected_joint, 'k', title + " torque command")
