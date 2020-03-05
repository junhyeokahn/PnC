import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

figure_number = 0

file_path = os.getcwd() + "/../../../ExperimentDataCheck/"

## read files
data_dcm_pos = \
np.genfromtxt(file_path+'dcm.txt', delimiter=None, dtype=(float))
data_dcm_pos_des = \
np.genfromtxt(file_path+'dcm_des.txt', delimiter=None, dtype=(float))

data_dcm_vel = \
np.genfromtxt(file_path+'dcm_vel.txt', delimiter=None, dtype=(float))
data_dcm_vel_des = \
np.genfromtxt(file_path+'dcm_vel_des.txt', delimiter=None, dtype=(float))

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

## plot dcm pos
fig = plt.figure(figure_number)
fig.canvas.set_window_title('dcm pos')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_dcm_pos[st_idx:end_idx,i-1], "b-")
    plt.plot(data_x, data_dcm_pos_des[st_idx:end_idx,i-1], "r-")
    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
plt.xlabel('time (sec)')

figure_number += 1
## plot dcm vel
fig = plt.figure(figure_number)
fig.canvas.set_window_title('dcm vel')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_dcm_vel[st_idx:end_idx,i-1], "b-")
    plt.plot(data_x, data_dcm_vel_des[st_idx:end_idx,i-1], "r-")
    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
plt.xlabel('time (sec)')

plt.show()
