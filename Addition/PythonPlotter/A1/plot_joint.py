import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

## -----------------------------------------------------------------------------
## Read Data
## -----------------------------------------------------------------------------
file_path = os.getcwd() + "/../../../ExperimentData/"

t = np.genfromtxt(file_path+'running_time.txt', delimiter='\n', dtype=(float))

st_idx = 40
end_idx = len(t) - 10
t = t[st_idx:end_idx]

tasks = dict()
tasks_des = dict()

final_config = np.genfromtxt(file_path+"final_config.txt", delimiter=None, dtype=(float))[st_idx:end_idx]
start_config = np.genfromtxt(file_path+"initial_config.txt", delimiter=None, dtype=(float))[st_idx:end_idx]
jpos_des = np.genfromtxt(file_path+"jpos_step_des.txt", delimiter=None, dtype=(float))[st_idx:end_idx]

data_phse = np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
phseChange = []
for i in range(0,len(t)-1):
        if data_phse[i] != data_phse[i+1]:
            phseChange.append(i)
        else:
            pass

def plot_phase(ax):
    for j in phseChange:
        ax.axvline(x=t[j],color='indigo',linestyle='-')
        ax.text(t[j],ax.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

fig, axes = plt.subplots(3, 4)
for i in range(3):
    # actual commanded value for the 4 legs
    axes[i, 0].plot(t, jpos_des[:,i], color='b', linestyle='solid', linewidth=2)
    axes[i, 1].plot(t, jpos_des[:,3+i], color='b', linestyle='solid', linewidth=2)
    axes[i, 2].plot(t, jpos_des[:,6+i], color='b', linestyle='solid', linewidth=2)
    axes[i, 3].plot(t, jpos_des[:,9+i], color='b', linestyle='solid', linewidth=2)
    # goal config for 4 legs
    axes[i, 0].plot(t, final_config[:,i], color='r', linestyle='dashed', linewidth=2)
    axes[i, 1].plot(t, final_config[:,i+3], color='r', linestyle='dashed', linewidth=2)
    axes[i, 2].plot(t, final_config[:,i+6], color='r', linestyle='dashed', linewidth=2)
    axes[i, 3].plot(t, final_config[:,i+9], color='r', linestyle='dashed', linewidth=2)
    # starting config for 4 legs
    axes[i, 0].plot(t, start_config[:,i], color='g', linestyle='dashed', linewidth=2)
    axes[i, 1].plot(t, start_config[:,i+3], color='g', linestyle='dashed', linewidth=2)
    axes[i, 2].plot(t, start_config[:,i+6], color='g', linestyle='dashed', linewidth=2)
    axes[i, 3].plot(t, start_config[:,i+9], color='g', linestyle='dashed', linewidth=2)
    # axes[i, 0].plot(t, jpos_max[i] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    # axes[i, 0].plot(t, jpos_min[i] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    # axes[i, 1].plot(t, jpos_max[i+3] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    # axes[i, 1].plot(t, jpos_min[i+3] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    # axes[i, 2].plot(t, jpos_max[i+6] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    # axes[i, 2].plot(t, jpos_min[i+6] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    # axes[i, 3].plot(t, jpos_max[i+9] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    # axes[i, 3].plot(t, jpos_min[i+9] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    axes[i, 0].grid(True)
    axes[i, 1].grid(True)
    axes[i, 2].grid(True)
    axes[i, 3].grid(True)
    plot_phase(axes[i,0])
    plot_phase(axes[i,1])
    plot_phase(axes[i,2])
    plot_phase(axes[i,3])


fig.suptitle("Joint Pos")

plt.show()
