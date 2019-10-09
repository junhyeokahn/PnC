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
data_rf_pos = \
np.genfromtxt(file_path+'rf_pos.txt', delimiter=None, dtype=(float))
data_rf_pos_des= \
np.genfromtxt(file_path+'rf_pos_des.txt', delimiter=None, dtype=(float))
data_rf_vel = \
np.genfromtxt(file_path+'rf_vel.txt', delimiter=None, dtype=(float))
data_rf_vel_des= \
np.genfromtxt(file_path+'rf_vel_des.txt', delimiter=None, dtype=(float))

data_lf_pos = \
np.genfromtxt(file_path+'lf_pos.txt', delimiter=None, dtype=(float))
data_lf_pos_des= \
np.genfromtxt(file_path+'lf_pos_des.txt', delimiter=None, dtype=(float))
data_lf_vel = \
np.genfromtxt(file_path+'lf_vel.txt', delimiter=None, dtype=(float))
data_lf_vel_des= \
np.genfromtxt(file_path+'lf_vel_des.txt', delimiter=None, dtype=(float))

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

# LeftFoot Contact Signal #
data_lf_contact = np.genfromtxt(file_path+'lfoot_contact.txt', delimiter=None, dtype=(float))
data_lf_contact = data_lf_contact[st_idx:end_idx]
lf_contact_index_change = []
lf_contact_index_change.append(0)
lf_contact_index_change.append(1)
for i in range(2, len(data_x)):
    if data_lf_contact[i] != data_lf_contact[i-1]:
        lf_contact_index_change.append(i)


# RightFoot Contact Signal #
data_rf_contact = np.genfromtxt(file_path+'rfoot_contact.txt', delimiter=None, dtype=(float))
data_rf_contact = data_rf_contact[st_idx:end_idx]
rf_contact_index_change = []
rf_contact_index_change.append(0)
rf_contact_index_change.append(1)
for i in range(2, len(data_x)):
    if data_rf_contact[i] != data_rf_contact[i-1]:
        rf_contact_index_change.append(i)

axes = plt.gca()

## plot rf pos
fig = plt.figure(figure_number)
fig.canvas.set_window_title('rf pos')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_rf_pos[st_idx:end_idx,i-1], "b-")
    plt.plot(data_x, data_rf_pos_des[st_idx:end_idx,i-1], "r-")

    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

    # Plot Left Foot Contact
    for j in range(1, len(lf_contact_index_change)):
        t_start = data_x[lf_contact_index_change[j-1]]
        t_end = data_x[lf_contact_index_change[j]]
        y_start = data_lf_contact[lf_contact_index_change[j-1]] * ax1.get_ylim()[1]/2.0 * 0.9
        y_end = data_lf_contact[lf_contact_index_change[j]] * ax1.get_ylim()[1]/2.0 * 0.9

        # Plot Square Wave
        plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='orange')
        plt.text((t_start+(t_end-t_start)/2.0), y_start,'L%d'%(data_lf_contact[lf_contact_index_change[j-1]]),color='orange')


    # Plot Right Foot Contact
    for j in range(1, len(rf_contact_index_change)):
        t_start = data_x[rf_contact_index_change[j-1]]
        t_end = data_x[rf_contact_index_change[j]]
        y_start = data_rf_contact[rf_contact_index_change[j-1]] * ax1.get_ylim()[1]/2.0
        y_end = data_rf_contact[rf_contact_index_change[j]] * ax1.get_ylim()[1]/2.0

        # Plot Square Wave
        plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='magenta')
        plt.text((t_start+(t_end-t_start)/2.0), y_start,'R%d'%(data_rf_contact[rf_contact_index_change[j-1]]),color='magenta')

plt.xlabel('time (sec)')

figure_number += 1
## plot rf vel
fig = plt.figure(figure_number)
fig.canvas.set_window_title('rf vel')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_rf_vel[st_idx:end_idx,i-1], "b-")
    plt.plot(data_x, data_rf_vel_des[st_idx:end_idx,i-1], "r-")

    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

plt.xlabel('time (sec)')

figure_number += 1
## plot lf pos
fig = plt.figure(figure_number)
fig.canvas.set_window_title('lf pos')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_lf_pos[st_idx:end_idx,i-1], "b-")
    plt.plot(data_x, data_lf_pos_des[st_idx:end_idx,i-1], "r-")

    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

    # Plot Left Foot Contact
    for j in range(1, len(lf_contact_index_change)):
        t_start = data_x[lf_contact_index_change[j-1]]
        t_end = data_x[lf_contact_index_change[j]]
        y_start = data_lf_contact[lf_contact_index_change[j-1]] * ax1.get_ylim()[1]/2.0 * 0.9
        y_end = data_lf_contact[lf_contact_index_change[j]] * ax1.get_ylim()[1]/2.0 * 0.9

        # Plot Square Wave
        plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='orange')
        plt.text((t_start+(t_end-t_start)/2.0), y_start,'L%d'%(data_lf_contact[lf_contact_index_change[j-1]]),color='orange')


    # Plot Right Foot Contact
    for j in range(1, len(rf_contact_index_change)):
        t_start = data_x[rf_contact_index_change[j-1]]
        t_end = data_x[rf_contact_index_change[j]]
        y_start = data_rf_contact[rf_contact_index_change[j-1]] * ax1.get_ylim()[1]/2.0
        y_end = data_rf_contact[rf_contact_index_change[j]] * ax1.get_ylim()[1]/2.0

        # Plot Square Wave
        plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='magenta')
        plt.text((t_start+(t_end-t_start)/2.0), y_start,'R%d'%(data_rf_contact[rf_contact_index_change[j-1]]),color='magenta')

plt.xlabel('time (sec)')

figure_number += 1
## plot lf vel
fig = plt.figure(figure_number)
fig.canvas.set_window_title('lf vel')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_lf_vel[st_idx:end_idx,i-1], "b-")
    plt.plot(data_x, data_lf_vel_des[st_idx:end_idx,i-1], "r-")

    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
plt.xlabel('time (sec)')

figure_number += 1

plt.show()
