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
data_r_rf= \
np.genfromtxt(file_path+'r_rf.txt', delimiter=None, dtype=(float))
data_l_rf= \
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

axes = plt.gca()

## plot r_rf
fig = plt.figure(figure_number)
fig.canvas.set_window_title('r rf')
for i in range(1,7,1):
    ax1 = plt.subplot(6, 1, i)
    plt.plot(data_x, data_r_rf[st_idx:end_idx,i-1], "b-")

    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

plt.xlabel('time (sec)')

figure_number += 1

## plot r_rf
fig = plt.figure(figure_number)
fig.canvas.set_window_title('l rf')
for i in range(1,7,1):
    ax1 = plt.subplot(6, 1, i)
    plt.plot(data_x, data_l_rf[st_idx:end_idx,i-1], "b-")

    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

plt.xlabel('time (sec)')

figure_number += 1

plt.show()
