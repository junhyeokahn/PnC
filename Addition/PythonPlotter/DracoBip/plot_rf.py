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

st_idx = 5
end_idx = len(t) - 10
t = t[st_idx:end_idx]

r_rf_front_des = np.genfromtxt(file_path+'r_rf_front_des.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
r_rf_back_des = np.genfromtxt(file_path+'r_rf_back_des.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
l_rf_front_des = np.genfromtxt(file_path+'l_rf_front_des.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
l_rf_back_des = np.genfromtxt(file_path+'l_rf_back_des.txt', delimiter=None, dtype=(float))[st_idx:end_idx]

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

fig, axes = plt.subplots(3, 4)
for i in range(3):
    axes[i,0].plot(t, r_rf_front_des[:,i], color='k', linewidth=3)
    axes[i,0].grid(True)
    plot_phase(axes[i,0])
    axes[i,1].plot(t, r_rf_back_des[:,i], color='k', linewidth=3)
    axes[i,1].grid(True)
    plot_phase(axes[i,1])
    axes[i,2].plot(t, l_rf_front_des[:,i], color='k', linewidth=3)
    axes[i,2].grid(True)
    plot_phase(axes[i,2])
    axes[i,3].plot(t, l_rf_back_des[:,i], color='k', linewidth=3)
    axes[i,3].grid(True)
    plot_phase(axes[i,3])

axes[0,0].set_title("Right Foot Toe")
axes[0,1].set_title("Right Foot Heel")
axes[0,2].set_title("Left Foot Toe")
axes[0,3].set_title("Left Foot Heel")

plt.show()
