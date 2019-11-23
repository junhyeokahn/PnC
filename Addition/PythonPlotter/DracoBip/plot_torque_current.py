import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

figure_number = 0

file_path = os.getcwd() + "/../../../ExperimentDataCheck/"

## read files
data_torque_msr = \
np.genfromtxt(file_path+'torque.txt', delimiter=None, dtype=(float))

data_torque_cmd = \
np.genfromtxt(file_path+'command.txt', delimiter=None, dtype=(float))

data_current = \
np.genfromtxt(file_path+'motor_current.txt', delimiter=None, dtype=(float))

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
fig.canvas.set_window_title('left leg current-torque')
for i in range(1,6,1):
    ax1 = plt.subplot(5, 1, i)
    plt.plot(data_current[st_idx:end_idx, i-1], data_torque_msr[st_idx:end_idx,i-1], "b-")
    plt.plot(data_current[st_idx:end_idx, i-1], data_torque_cmd[st_idx:end_idx,i-1], "r-")

    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
plt.xlabel('current (A)')

figure_number += 1

fig = plt.figure(figure_number)
fig.canvas.set_window_title('right current-torque')
for i in range(1,6,1):
    ax1 = plt.subplot(5, 1, i)
    plt.plot(data_current[st_idx:end_idx, i+5-1], data_torque_msr[st_idx:end_idx,i+5-1], "b-")
    plt.plot(data_current[st_idx:end_idx, i+5-1], data_torque_cmd[st_idx:end_idx,i+5-1], "r-")

    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
plt.xlabel('current (A)')

plt.show()
