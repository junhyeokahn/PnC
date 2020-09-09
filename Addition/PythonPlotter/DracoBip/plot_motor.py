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

temperature = np.genfromtxt(file_path+'temperature.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
current = np.genfromtxt(file_path+'motor_current.txt', delimiter=None, dtype=(float))[st_idx:end_idx]

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

fig, axes = plt.subplots(5, 2)
for i in range(5):
    axes[i,0].plot(t, temperature[:,i], color='k', linewidth=3)
    axes[i,1].plot(t, temperature[:,i+5], color='k', linewidth=3)
    for j in range(2):
        plot_phase(axes[i,j])
        axes[i,j].grid(True)
fig.suptitle("Temperature")

fig, axes = plt.subplots(5, 2)
for i in range(5):
    axes[i,0].plot(t, current[:,i], color='k', linewidth=3)
    axes[i,1].plot(t, current[:,i+5], color='k', linewidth=3)
    for j in range(2):
        plot_phase(axes[i,j])
        axes[i,j].grid(True)
fig.suptitle("Motor Current")

plt.show()
