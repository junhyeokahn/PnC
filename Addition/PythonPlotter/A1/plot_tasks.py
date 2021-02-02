import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

## -----------------------------------------------------------------------------
## Read Data
## -----------------------------------------------------------------------------
file_path = os.getcwd() + "/../../../ExperimentDataCheck/"

t = np.genfromtxt(file_path+'running_time.txt', delimiter='\n', dtype=(float))

st_idx = 40
end_idx = len(t) - 10
t = t[st_idx:end_idx]

task_names = ['com_pos', 'com_vel']
task_labels = ['com lin']
tasks = dict()
tasks_des = dict()
for i, label in enumerate(task_labels):
    p_d = np.genfromtxt(file_path+task_names[2*i]+"_des.txt", delimiter=None, dtype=(float))[st_idx:end_idx]
    v_d = np.genfromtxt(file_path+task_names[2*i+1]+"_des.txt", delimiter=None, dtype=(float))[st_idx:end_idx]
    p = np.genfromtxt(file_path+task_names[2*i]+".txt", delimiter=None, dtype=(float))[st_idx:end_idx]
    v = np.genfromtxt(file_path+task_names[2*i+1]+".txt", delimiter=None, dtype=(float))[st_idx:end_idx]
    tasks_des[label] = [p_d, v_d]
    tasks[label] = [p, v]
com_vel = np.genfromtxt(file_path+"com_vel.txt", delimiter=None, dtype=(float))[st_idx:end_idx]

data_phse = np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
phseChange = []
for i in range(0,len(t)-1):
        if data_phse[i] != data_phse[i+1]:
            phseChange.append(i)
        else:
            pass

## -----------------------------------------------------------------------------
## Plot Tasks
## -----------------------------------------------------------------------------
def plot_phase(ax):
    for j in phseChange:
        ax.axvline(x=t[j],color='indigo',linestyle='-')
        ax.text(t[j],ax.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

for (k_des, v_des), (k, v) in zip(tasks_des.items(), tasks.items()):
    dim = v_des[0].shape[1]
    if dim == 4:
        fig, axes = plt.subplots(dim, 2)
        for i in range(dim):
            for j in range(2):
                if not((i == dim-1) and (j == 1)):
                    axes[i,j].plot(t, v_des[j][:,i], color='r', linestyle='dashed', linewidth=4)
                    axes[i,j].plot(t, v[j][:,i], color='b', linestyle='solid', linewidth=2)
                    plot_phase(axes[i,j])
                    axes[i,j].grid(True)
        axes[3, 0].set_xlabel('time')
        axes[2, 1].set_xlabel('time')
        fig.suptitle(k)
    else:
        fig, axes = plt.subplots(dim, 2)
        for i in range(dim):
            for j in range(2):
                axes[i,j].plot(t, v[j][:,i], color='b', linestyle='solid', linewidth=2)
                if k=='com lin':
                    axes[i,1].plot(t, com_vel[:,i], color='g', linestyle='solid', linewidth=2)
                axes[i,j].plot(t, v_des[j][:,i], color='r', linestyle='dashed', linewidth=4)
                plot_phase(axes[i,j])
                axes[i,j].grid(True)
            axes[dim-1, j].set_xlabel('time')
        fig.suptitle(k)

plt.show()
