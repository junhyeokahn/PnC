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

st_idx = 5
end_idx = len(t) - 10
t = t[st_idx:end_idx]

foot_force = np.genfromtxt(file_path+'foot_force.txt', delimiter=None, dtype=(float))[st_idx:end_idx]

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

fig, axes = plt.subplots(1, 4)

axes[0].plot(t, foot_force[:,0], color='r', linestyle='dashed', linewidth=2)
axes[0].set_ylabel('Force')
axes[1].plot(t, foot_force[:,1], color='r', linestyle='dashed', linewidth=2)
axes[1].set_ylabel('Force')
axes[2].plot(t, foot_force[:,2], color='r', linestyle='dashed', linewidth=2)
axes[2].set_ylabel('Force')
axes[3].plot(t, foot_force[:,3], color='r', linestyle='dashed', linewidth=2)
axes[3].set_ylabel('Force')

axes[0].set_title('FL foot')
axes[1].set_title('FR foot')
axes[2].set_title('RL foot')
axes[3].set_title('RR foot')

for i in range(4):   
    axes[i].grid(True)
    plot_phase(axes[i])

plt.show()


