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

task_names = ['w_com', 'w_flfoot_pos', 'w_frfoot_pos', 'w_rlfoot_pos', 'w_rrfoot_pos']
task_labels = ['com', 'flfoot pos', 'frfoot pos', 'rlfoot pos', 'rrfoot pos']
task_weights = dict()
for label, task_name in zip(task_labels, task_names):
    task_weights[label] = np.genfromtxt(file_path+task_name+".txt", delimiter='\n', dtype=(float))[st_idx:end_idx]

contact_names = ['w_frfoot_fr', 'w_flfoot_fr', 'w_rrfoot_fr', 'w_rlfoot_fr']
contact_labels= ['Front Right Foot', 'Front Left Foot', 'Rear Right Foot', 'Rear Left Foot']
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
