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

config = np.genfromtxt(file_path+'config.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
config2 = np.genfromtxt(file_path2+'config.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
qdot = np.genfromtxt(file_path+'qdot.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
qdot2 = np.genfromtxt(file_path2+'qdot.txt', delimiter=None, dtype=(float))[st_idx:end_idx]

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

fig, axes = plt.subplots(6, 2)
for i in range(6):
    axes[i,0].plot(t, config[:,i], color='r', linewidth=3)
    axes[i,0].plot(t2, config2[:,i], color='k', linewidth=3)
    axes[i,0].grid(True)
    plot_phase(axes[i,0])
    axes[i,1].plot(t, qdot[:,i], color='r', linewidth=3)
    axes[i,1].plot(t2, qdot2[:,i], color='k', linewidth=3)
    axes[i,1].grid(True)
    plot_phase(axes[i,1])

axes[0,0].set_title("q0 ~ q5")
axes[0,1].set_title("qdot0 ~ qdot5")

fig, axes = plt.subplots(5, 2)
for i in range(5):
    axes[i,0].plot(t, config[:,i+6], color='r', linewidth=3)
    axes[i,0].plot(t2, config2[:,i+6], color='k', linewidth=3)
    axes[i,0].grid(True)
    plot_phase(axes[i,0])
    axes[i,1].plot(t, config[:,i+11], color='r', linewidth=3)
    axes[i,1].plot(t2, config2[:,i+11], color='k', linewidth=3)
    axes[i,1].grid(True)
    plot_phase(axes[i,1])

axes[0,0].set_title("q6 ~ q10")
axes[0,1].set_title("q11 ~ q15")

## axes[0,0].set(xlim=(xmin, xmax), ylim=(ymin, ymax))

fig, axes = plt.subplots(5, 2)
for i in range(5):
    axes[i,0].plot(t, qdot[:,i+6], color='r', linewidth=3)
    axes[i,0].plot(t2, qdot2[:,i+6], color='k', linewidth=3)
    axes[i,0].grid(True)
    plot_phase(axes[i,0])
    axes[i,1].plot(t, qdot[:,i+11], color='r', linewidth=3)
    axes[i,1].plot(t2, qdot2[:,i+11], color='k', linewidth=3)
    axes[i,1].grid(True)
    plot_phase(axes[i,1])

axes[0,0].set_title("qdot6 ~ qdot10")
axes[0,1].set_title("qdot11 ~ qdot15")

plt.show()
