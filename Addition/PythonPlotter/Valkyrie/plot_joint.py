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
data_q = \
np.genfromtxt(file_path+'q.txt', delimiter=None, dtype=(float))
data_jpos_des = \
np.genfromtxt(file_path+'jpos_des.txt', delimiter=None, dtype=(float))

data_qdot = \
np.genfromtxt(file_path+'qdot.txt', delimiter=None, dtype=(float))
data_jvel_des = \
np.genfromtxt(file_path+'jvel_des.txt', delimiter=None, dtype=(float))

command = \
np.genfromtxt(file_path+'command.txt', delimiter=None, dtype=(float))

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

jidx = range(data_jpos_des.shape[1])

for i in jidx:
    fig=plt.figure(figure_number)
    fig.canvas.set_window_title(str(i) + ' th joint')

    ax = plt.subplot(3, 1, 1)
    plt.plot(data_x, data_q[st_idx:end_idx, i+6], "b-")
    plt.plot(data_x, data_jpos_des[st_idx:end_idx, i], "r-")
    plt.grid(True)

    ax = plt.subplot(3, 1, 2)
    plt.plot(data_x, data_qdot[st_idx:end_idx, i+6], "b-")
    plt.plot(data_x, data_jvel_des[st_idx:end_idx, i], "r-")
    plt.grid(True)

    ax = plt.subplot(3, 1, 3)
    plt.plot(data_x, command[st_idx:end_idx, i], "b-")
    plt.grid(True)

    plt.xlabel('time (sec)')

    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

    figure_number += 1

plt.show()
