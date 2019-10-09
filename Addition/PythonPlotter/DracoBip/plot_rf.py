import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

figure_number = 0

file_path = os.getcwd() + "/../../../ExperimentDataCheck/"

## read files
data_r_rf_des = \
np.genfromtxt(file_path+'r_rf_des.txt', delimiter=None, dtype=(float))
data_l_rf_des = \
np.genfromtxt(file_path+'l_rf_des.txt', delimiter=None, dtype=(float))
data_r_rf = \
np.genfromtxt(file_path+'r_rf.txt', delimiter=None, dtype=(float))
data_l_rf = \
np.genfromtxt(file_path+'l_rf.txt', delimiter=None, dtype=(float))

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

## plot r_rf
fig = plt.figure()
fig.canvas.set_window_title('right foot wrench')
for i in range(1,7,1):
    ax1 = plt.subplot(6, 1, i)
    plt.plot(data_x, data_r_rf[st_idx:end_idx,i-1], "b-")
    plt.plot(data_x, data_r_rf_des[st_idx:end_idx,i-1], "r-")

    plt.grid(True)
    if i == 6:
        plt.ylim(0., 1500.)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

plt.xlabel('time (sec)')

## plot l_lf
fig = plt.figure()
fig.canvas.set_window_title('left foot wrench')
for i in range(1,7,1):
    ax1 = plt.subplot(6, 1, i)
    plt.plot(data_x, data_l_rf[st_idx:end_idx,i-1], "b-")
    plt.plot(data_x, data_l_rf_des[st_idx:end_idx,i-1], "r-")

    plt.grid(True)
    if i == 6:
        plt.ylim(0., 1500.)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

plt.xlabel('time (sec)')

plt.show()
