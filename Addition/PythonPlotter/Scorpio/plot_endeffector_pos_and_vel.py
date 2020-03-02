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
data_act_pos = \
np.genfromtxt(file_path+'act_pos.txt', delimiter=None, dtype=(float))
data_des_pos = \
np.genfromtxt(file_path+'des_pos.txt', delimiter=None, dtype=(float))

data_act_vel = \
np.genfromtxt(file_path+'act_vel.txt', delimiter=None, dtype=(float))
data_des_vel= \
np.genfromtxt(file_path+'des_vel.txt', delimiter=None, dtype=(float))

data_ori_err = \
np.genfromtxt(file_path+'ori_error.txt', delimiter=None, dtype=(float))

data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))
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

## plot end-effector pos
fig = plt.figure(figure_number)
fig.canvas.set_window_title('end-effector pos')
for i in range(1,8,1):
    ax1 = plt.subplot(7, 1, i)
    plt.plot(data_x, data_act_pos[st_idx:end_idx,i-1], "b-")
    plt.plot(data_x, data_des_pos[st_idx:end_idx,i-1], "r-")

    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
plt.xlabel('time (sec)')

figure_number += 1
## plot end_effector vel
fig = plt.figure(figure_number)
fig.canvas.set_window_title('end_effector vel')
for i in range(1,7,1):
    ax1 = plt.subplot(6, 1, i)
    plt.plot(data_x, data_act_vel[st_idx:end_idx,i-1], "b-")
    plt.plot(data_x, data_des_vel[st_idx:end_idx,i-1], "r-")

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
