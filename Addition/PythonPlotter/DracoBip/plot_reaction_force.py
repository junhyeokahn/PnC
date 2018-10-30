import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os


file_path = os.getcwd() + "/../../../experiment_data_check/"
dim_single_rf = 5
## read files
data_rf_cmd = \
np.genfromtxt(file_path+'reaction_force.txt', delimiter=None, dtype=(float))
data_rf_sense = \
np.genfromtxt(file_path+'reaction_force.txt', delimiter=None, dtype=(float))
data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))

st_idx = 10;
end_idx = len(data_x) - 10;
data_x = data_x[st_idx:end_idx];
# PHASE MARKER #
data_phse = np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))
# get phase.txt data #
phseChange = []
for i in range(0,len(data_x)-1):
        if data_phse[i] != data_phse[i+1]:
            phseChange.append(i+1 - st_idx)
        else:
            pass
axes = plt.gca()

## plot command/jpos
fig = plt.figure(1)
plt.get_current_fig_manager().window.wm_geometry("480x600+0+0")
fig.canvas.set_window_title('reaction_force (right_leg)')
for i in range(1, dim_single_rf + 1,1):
    ax1 = plt.subplot(dim_single_rf, 1, i)
    plt.plot(data_x, data_rf_cmd[st_idx:end_idx,i-1], "r-", \
             data_x, data_rf_sense[st_idx:end_idx,i-1], "b-")
    # plt.legend(('command', 'pos'), loc='upper left')
    # phase marker #
    for j in phseChange:
        # phase line
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        # phase number
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
    plt.grid(True)
plt.xlabel('time (sec)')

fig = plt.figure(2)
plt.get_current_fig_manager().window.wm_geometry("480x600+480+0")
fig.canvas.set_window_title('reaction_force (left_leg)')
for i in range(1,dim_single_rf + 1,1):
    ax1 = plt.subplot(dim_single_rf, 1, i)
    plt.plot(data_x, data_rf_cmd[st_idx:end_idx,i-1 + dim_single_rf], "r-" , \
             data_x, data_rf_sense[st_idx:end_idx,i-1 + dim_single_rf], "b-")
    # phase marker #
    for j in phseChange:
        # phase line
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        # phase number
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')

plt.show()

