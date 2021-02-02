import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

## -----------------------------------------------------------------------------
## Read Data
## -----------------------------------------------------------------------------
file_path = os.getcwd() + "/../../../ExperimentDataCheck/"

t = np.genfromtxt(file_path+'running_time.txt', delimiter='\n', dtype=(float))

st_time = 9 
end_time = 16

for i in range(0,len(t)-2):
    if t[i] < st_time and t[i+1] >= st_time:
        st_idx = i 
    elif t[i] < end_time and t[i+1] >= end_time:
        end_idx = i 
           
t_modified = t[st_idx:end_idx]

r_front_rf_ati = np.genfromtxt(file_path+'rfoot_front_ati.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
r_back_rf_ati = np.genfromtxt(file_path+'rfoot_back_ati.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
l_front_rf_ati = np.genfromtxt(file_path+'lfoot_front_ati.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
l_back_rf_ati = np.genfromtxt(file_path+'lfoot_back_ati.txt', delimiter=None, dtype=(float))[st_idx:end_idx]

r_front_rf_est = np.genfromtxt(file_path+'rfoot_front_est.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
r_back_rf_est = np.genfromtxt(file_path+'rfoot_back_est.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
l_front_rf_est = np.genfromtxt(file_path+'lfoot_front_est.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
l_back_rf_est = np.genfromtxt(file_path+'lfoot_back_est.txt', delimiter=None, dtype=(float))[st_idx:end_idx]

F_ext = np.genfromtxt(file_path+'Fr_ext.txt', delimiter=None, dtype=(float))[st_idx:end_idx]

y_label = ["Fx", "Fy", "Fz"]

# data_phse = np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
# phseChange = []
# for i in range(0,len(t)-1):
#         if data_phse[i] != data_phse[i+1]:
#             phseChange.append(i)
#         else:
#             pass

## -----------------------------------------------------------------------------
## Plot Cmd
## -----------------------------------------------------------------------------
# def plot_phase(ax):
#     for j in phseChange:
#         ax.axvline(x=t[j],color='indigo',linestyle='-')
#         ax.text(t[j],ax.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

fig, axes = plt.subplots(3, 4)
# plt.figure()
for i in range(3):
    axes[i,0].plot(t_modified, r_front_rf_ati[:,i+3], color='k', linewidth=2)
    axes[i,0].plot(t_modified, r_front_rf_est[:,i+3], color='r', linewidth=2, linestyle='dashed')
    axes[i,0].grid(True)
    axes[i,0].set_ylabel(y_label[i])
    if i == 2:
        axes[i,0].set_ylim([0, 250])
    else:
        axes[i,0].set_ylim([-10, 10])
    # plot_phase(axes[i,0])
    axes[i,1].plot(t_modified, r_back_rf_ati[:,i+3], color='k', linewidth=2)
    axes[i,1].plot(t_modified, r_back_rf_est[:,i+3], color='r', linewidth=2, linestyle='dashed')
    axes[i,1].grid(True)
    if i == 2:
        axes[i,1].set_ylim([0, 250])
    else:
        axes[i,1].set_ylim([-10, 10])
    # plot_phase(axes[i,1])
    axes[i,2].plot(t_modified, l_front_rf_ati[:,i+3], color='k', linewidth=2)
    axes[i,2].plot(t_modified, l_front_rf_est[:,i+3], color='r', linewidth=2, linestyle='dashed')
    axes[i,2].grid(True)
    if i == 2:
        axes[i,2].set_ylim([0, 250])
    else:
        axes[i,2].set_ylim([-10, 10])
    # plot_phase(axes[i,2])    
    axes[i,3].plot(t_modified, l_back_rf_ati[:,i+3], color='k', linewidth=2)
    axes[i,3].plot(t_modified, l_back_rf_est[:,i+3], color='r', linewidth=2, linestyle='dashed')
    axes[i,3].grid(True)
    if i == 2:
        axes[i,3].set_ylim([0, 250])
    else:
        axes[i,3].set_ylim([-10, 10])
    # plot_phase(axes[i,2])    


axes[0,0].set_title("Right Front Foot")
axes[0,1].set_title("Right Back Foot")
axes[0,2].set_title("Left Front Foot")
axes[0,3].set_title("Left Back Foot")

# fig, axes = plt.subplots(3, 1)
fig, axes = plt.subplots(3,2)
# plt.figure()
for i in range(3):
    axes[i,0].plot(t_modified, F_ext[:,i], color='k', linewidth=3)
    # plot_phase(axes[i, 0])
    axes[i,0].grid(True)
    axes[i,0].set_ylim([-10, 10])
    axes[i,1].plot(t_modified, F_ext[:,i+3], color='k', linewidth=3)
    # plot_phase(axes[i, 0])
    axes[i,1].grid(True)
    axes[i,1].set_ylim([-10, 10])
    # axes[i,4].set_ylabel(y_label[i])
axes[0,0].set_title("F_ext")

plt.show()
