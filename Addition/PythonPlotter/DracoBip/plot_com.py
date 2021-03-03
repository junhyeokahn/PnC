import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

## -----------------------------------------------------------------------------
## Read Data
## -----------------------------------------------------------------------------
file_path = os.getcwd() + "/../../../WithoutSim/"
file_path2 = os.getcwd() + "/../../../WithSim/"

t = np.genfromtxt(file_path+'running_time.txt', delimiter='\n', dtype=(float))
t2 = np.genfromtxt(file_path2+'running_time.txt', delimiter='\n', dtype=(float))


if len(t) > len(t2):
    end_idx = len(t) - 10
else:
    end_idx = len(t2) - 10

st_idx = 5
t = t[st_idx:end_idx]
t2 = t2[st_idx:end_idx]

com1 = np.genfromtxt(file_path+'com_pos.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
com2 = np.genfromtxt(file_path2+'com_pos.txt', delimiter=None, dtype=(float))[st_idx:end_idx]

com_vel1 = np.genfromtxt(file_path+'com_vel.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
com_vel2 = np.genfromtxt(file_path2+'com_vel.txt', delimiter=None, dtype=(float))[st_idx:end_idx]


data_phse = np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
phseChange = []
for i in range(0,len(t)-1):
        if data_phse[i] != data_phse[i+1]:
            phseChange.append(i)
        else:
            pass

## -----------------------------------------------------------------------------
## Plot Cmd
## -----------------------------------------------------------------------------
def plot_phase(ax):
    for j in phseChange:
        ax.axvline(x=t[j],color='indigo',linestyle='-')
        ax.text(t[j],ax.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

fig, axes = plt.subplots(3, 2)
for i in range(3):
    axes[i,0].plot(t, com1[:,i], color='r', linewidth=3)
    axes[i,0].plot(t2, com2[:,i], color='k', linewidth=3)
    axes[i,0].grid(True)
    plot_phase(axes[i,0])
    axes[i,1].plot(t, com_vel1[:,i], color='r', linewidth=3)
    axes[i,1].plot(t2, com_vel2[:,i], color='k', linewidth=3)
    axes[i,1].grid(True)
    plot_phase(axes[i,1])

plt.show()

