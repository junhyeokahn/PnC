import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

## -----------------------------------------------------------------------------
## Read Data
## -----------------------------------------------------------------------------
file_path = os.getcwd() + "/../../../ExperimentDataCheck/"

st_idx_offset = 5
end_idx_offset = 10
file_name = ['debug_wbc_rf', 'debug_wbc_task_weight', 'debug_wbc_uf_ieq', \
        'debug_wbc_qddot', 'debug_wbc_tau_cmd', 'debug_wbc_task_0', \
        'debug_wbc_task_1', 'debug_wbc_task_2', 'debug_wbc_task_3',
        'debug_wbc_task_4', 'debug_wbc_task_5']
data = list()
for i in file_name:
    data.append(np.genfromtxt(file_path+i+'.txt', delimiter=None, dtype=(float))[st_idx_offset:-end_idx_offset])

## -----------------------------------------------------------------------------
## Plot
## -----------------------------------------------------------------------------

for j in range(len(data)):
    fig, axes = plt.subplots(data[j].shape[1])
    for i in range(data[j].shape[1]):
        axes[i].plot(data[j][:,i], color='k', linewidth=3)
        axes[i].grid(True)
    fig.suptitle(file_name[j])

plt.show()
