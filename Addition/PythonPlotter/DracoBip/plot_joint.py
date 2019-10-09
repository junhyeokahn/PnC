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

data_des_jacc_cmd = \
np.genfromtxt(file_path+'des_jacc_cmd.txt', delimiter=None, dtype=(float))

data_command = \
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

left_leg_idx = [0, 1, 2, 3, 4] ## left leg
right_leg_idx = [5, 6, 7, 8, 9] ## right leg
n_leg = len(left_leg_idx)

fig=plt.figure(figure_number)
fig.canvas.set_window_title('left leg jpos(hy, hr, hp, kp, ap)')
for i in range(1, 6, 1):
    ax = plt.subplot(5, 1, i)
    plt.plot(data_x, data_q[st_idx:end_idx, left_leg_idx[i-1]+6], 'b-')
    plt.plot(data_x, data_jpos_des[st_idx:end_idx, left_leg_idx[i-1]], 'r-')
    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
figure_number += 1

fig=plt.figure(figure_number)
fig.canvas.set_window_title('left leg jvel(hy, hr, hp, kp, ap)')
for i in range(1, 6, 1):
    ax = plt.subplot(5, 1, i)
    plt.plot(data_x, data_qdot[st_idx:end_idx, left_leg_idx[i-1]+6], 'b-')
    plt.plot(data_x, data_jvel_des[st_idx:end_idx, left_leg_idx[i-1]], 'r-')
    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
figure_number += 1

fig=plt.figure(figure_number)
fig.canvas.set_window_title('left leg command(hy, hr, hp, kp, ap, ar)')
for i in range(1, 6, 1):
    ax = plt.subplot(5, 1, i)
    plt.plot(data_x, data_command[st_idx:end_idx, left_leg_idx[i-1]], 'b-')
    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
figure_number += 1


fig=plt.figure(figure_number)
fig.canvas.set_window_title('right leg jpos(hy, hr, hp, kp, ap, ar)')
for i in range(1, 6, 1):
    ax = plt.subplot(5, 1, i)
    plt.plot(data_x, data_q[st_idx:end_idx, right_leg_idx[i-1]+6], 'b-')
    plt.plot(data_x, data_jpos_des[st_idx:end_idx, right_leg_idx[i-1]], 'r-')
    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
figure_number += 1

fig=plt.figure(figure_number)
fig.canvas.set_window_title('right leg jvel(hy, hr, hp, kp, ap, ar)')
for i in range(1, 6, 1):
    ax = plt.subplot(5, 1, i)
    plt.plot(data_x, data_qdot[st_idx:end_idx, right_leg_idx[i-1]+6], 'b-')
    plt.plot(data_x, data_jvel_des[st_idx:end_idx, right_leg_idx[i-1]], 'r-')
    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
figure_number += 1

fig=plt.figure(figure_number)
fig.canvas.set_window_title('right leg command(hy, hr, hp, kp, ap, ar)')
for i in range(1, 6, 1):
    ax = plt.subplot(5, 1, i)
    plt.plot(data_x, data_command[st_idx:end_idx, right_leg_idx[i-1]], 'b-')
    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
figure_number += 1

fig=plt.figure(figure_number)
fig.canvas.set_window_title('right leg des_jacc_cmd(hy, hr, hp, kp, ap, ar)')
for i in range(1, 6, 1):
    ax = plt.subplot(5, 1, i)
    plt.plot(data_x, data_des_jacc_cmd[st_idx:end_idx, right_leg_idx[i-1]], 'b-')
    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
figure_number += 1

fig=plt.figure(figure_number)
fig.canvas.set_window_title('left leg des_jacc_cmd(hy, hr, hp, kp, ap, ar)')
for i in range(1, 6, 1):
    ax = plt.subplot(5, 1, i)
    plt.plot(data_x, data_des_jacc_cmd[st_idx:end_idx, left_leg_idx[i-1]], 'b-')
    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
figure_number += 1

plt.show()
