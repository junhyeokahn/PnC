import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

figure_number = 0
col_index = 0
row_index = 0

file_path = os.getcwd() + "/../../../ExperimentDataCheck/"

## read files
data_act_ori = \
np.genfromtxt(file_path+'act_ori_data.txt', delimiter=None, dtype=(float))
data_des_ori = \
np.genfromtxt(file_path+'des_ori_data.txt', delimiter=None, dtype=(float))

data_ori_err = \
np.genfromtxt(file_path+'ori_err_so3.txt', delimiter=None, dtype=(float))

data_x = np.genfromtxt(file_path+'state_machine_time.txt', delimiter='\n', dtype=(float))

st_idx = 1
end_idx = len(data_x) - 10
data_x = data_x[st_idx:end_idx]

data_phse = np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))
data_phse = data_phse[st_idx:end_idx]
phseChange = []
for i in range(0,len(data_x)-1):
        if data_phse[i] != data_phse[i+1]:
            phseChange.append(i)
        else:
            pass
axes = plt.gca()

## plot end-effector ori
fig = plt.figure(figure_number)
fig.canvas.set_window_title('end-effector ori')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_act_ori[st_idx:end_idx,i-1], "b-")
    plt.plot(data_x, data_des_ori[st_idx:end_idx,i-1], "r-")

    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
plt.xlabel('time (sec)')

figure_number += 1

# plot ori_error
fig = plt.figure(figure_number)
fig.canvas.set_window_title('ori_error')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_ori_err[st_idx:end_idx,i-1], "r-")

    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
plt.xlabel('time (sec)')

plt.show()
