import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

figure_number = 0

file_path = os.getcwd() + "/../../../ExperimentDataCheck/"

## read files
data_com_pos = \
np.genfromtxt(file_path+'com_pos.txt', delimiter=None, dtype=(float))
data_com_pos_des = \
np.genfromtxt(file_path+'com_pos_des.txt', delimiter=None, dtype=(float))

data_dcm_pos = \
np.genfromtxt(file_path+'dcm.txt', delimiter=None, dtype=(float))

data_mpc_pred_pos = \
np.genfromtxt(file_path+'mpc_pred_pos.txt', delimiter=None, dtype=(float))

data_com_vel = \
np.genfromtxt(file_path+'com_vel.txt', delimiter=None, dtype=(float))
data_est_com_vel = \
np.genfromtxt(file_path+'est_com_vel.txt', delimiter=None, dtype=(float))
data_com_vel_des = \
np.genfromtxt(file_path+'com_vel_des.txt', delimiter=None, dtype=(float))

data_mpc_pred_vel = \
np.genfromtxt(file_path+'mpc_pred_vel.txt', delimiter=None, dtype=(float))

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

## plot com pos
fig = plt.figure(figure_number)
fig.canvas.set_window_title('com pos')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_com_pos[st_idx:end_idx,i-1], "b-")
    plt.plot(data_x, data_com_pos_des[st_idx:end_idx,i-1], "r-")
    plt.plot(data_x, data_mpc_pred_pos[st_idx:end_idx,i-1], "k-")
    plt.plot(data_x, data_dcm_pos[st_idx:end_idx,i-1], "c-")
    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
plt.xlabel('time (sec)')

figure_number += 1
## plot com vel
fig = plt.figure(figure_number)
fig.canvas.set_window_title('com vel')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_com_vel[st_idx:end_idx,i-1], "b-")
    plt.plot(data_x, data_est_com_vel[st_idx:end_idx,i-1], "g-")
    plt.plot(data_x, data_com_vel_des[st_idx:end_idx,i-1], "r-")
    plt.plot(data_x, data_mpc_pred_vel[st_idx:end_idx,i-1], "k-")
    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
plt.xlabel('time (sec)')

plt.show()
