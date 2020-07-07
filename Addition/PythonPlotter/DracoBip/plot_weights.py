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

task_names = ['w_com', 'w_lfoot_pos', 'w_lfoot_ori', 'w_rfoot_pos', 'w_rfoot_ori']
task_labels = ['com', 'lfoot pos', 'lfoot ori', 'rfoot pos', 'rfoot ori']
task_weights = dict()
for label, task_name in zip(task_labels, task_names):
    task_weights[label] = np.genfromtxt(file_path+task_name+".txt", delimiter='\n', dtype=(float))[st_idx:end_idx]

contact_names = ['w_rf_rffront', 'w_rf_rfback', 'w_rf_lffront', 'w_rf_lfback']
contact_labels= ['rfoot front', 'rfoot back', 'lfoot front', 'lfoot back']
contact_weights = dict()
for label, contact_name in zip(contact_labels, contact_names):
    contact_weights[label] = np.genfromtxt(file_path+contact_name+".txt", delimiter='\n', dtype=(float))[st_idx:end_idx]

data_phse = np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
phseChange = []
for i in range(0,len(t)-1):
        if data_phse[i] != data_phse[i+1]:
            phseChange.append(i)
        else:
            pass

## -----------------------------------------------------------------------------
## Plot Task Weights
## -----------------------------------------------------------------------------

def plot_phase(ax):
    for j in phseChange:
        ax.axvline(x=t[j],color='indigo',linestyle='-')
        ax.text(t[j],ax.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

fig, ax = plt.subplots()
for key, value in task_weights.items():
    ax.plot(t, value, label=key, linewidth=3)
plot_phase(ax)
ax.set_ylabel('task weight')
ax.set_xlabel('time')
ax.grid(True)
plt.legend()

## -----------------------------------------------------------------------------
## Plot Contact Weights
## -----------------------------------------------------------------------------
fig, ax = plt.subplots()
for key, value in contact_weights.items():
    ax.plot(t, value, label=key, linewidth=3)
plot_phase(ax)
ax.set_ylabel('contact weight')
ax.set_xlabel('time')
ax.grid(True)
plt.legend()
plt.show()
