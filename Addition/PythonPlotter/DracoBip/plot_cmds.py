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

st_idx = 5
end_idx = len(t) - 10
t = t[st_idx:end_idx]

jpos_cmd = np.genfromtxt(file_path+"jpos_des.txt", delimiter=None, dtype=(float))[st_idx:end_idx]
jvel_cmd = np.genfromtxt(file_path+"jvel_des.txt", delimiter=None, dtype=(float))[st_idx:end_idx]
trq_cmd = np.genfromtxt(file_path+"command.txt", delimiter=None, dtype=(float))[st_idx:end_idx]
q = np.genfromtxt(file_path+'config.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
qdot = np.genfromtxt(file_path+'qdot.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
qddot_des = np.genfromtxt(file_path+'qddot_des.txt', delimiter=None, dtype=(float))[st_idx:end_idx]

jpos_max = np.array([0.6  , 0.4  , 0.5   , 1.9   , 2.33 , 0.6  , 0.4  , 0.5   , 1.9   , 2.33])
jpos_min = np.array([-0.6 , -0.4 , -1.15 , -0.2  , 0.64 , -0.6 , -0.4 , -1.15 , -0.2  , 0.64])
jvel_max = np.array([10., 10., 10., 10., 10., 10., 10., 10., 10., 10.])
jtrq_max = np.array([200.  , 200.  , 200.  , 200.  , 100.  , 200.  , 200.  , 200.  , 200.  , 100.])

data_phse = np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
phseChange = []
for i in range(0,len(t)-1):
        if data_phse[i] != data_phse[i+1]:
            phseChange.append(i)
        else:
            pass

## -----------------------------------------------------------------------------
## Plot Cmd
## -----------------------------------------------------------------------------
def plot_phase(ax):
    for j in phseChange:
        ax.axvline(x=t[j],color='indigo',linestyle='-')
        ax.text(t[j],ax.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

fig, axes = plt.subplots(5, 2)
for i in range(5):
    axes[i, 0].plot(t, q[:,6+i], color='b', linestyle='solid', linewidth=2)
    axes[i, 1].plot(t, q[:,11+i], color='b', linestyle='solid', linewidth=2)
    axes[i, 0].plot(t, jpos_cmd[:,i], color='r', linestyle='dashed', linewidth=4)
    axes[i, 1].plot(t, jpos_cmd[:,i+5], color='r', linestyle='dashed', linewidth=4)
    axes[i, 0].plot(t, jpos_max[i] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    axes[i, 0].plot(t, jpos_min[i] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    axes[i, 1].plot(t, jpos_max[i+5] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    axes[i, 1].plot(t, jpos_min[i+5] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    axes[i, 0].grid(True)
    axes[i, 1].grid(True)
    plot_phase(axes[i,0])
    plot_phase(axes[i,1])
fig.suptitle("Joint Pos")


fig, axes = plt.subplots(5, 2)
for i in range(5):
    axes[i, 0].plot(t, qdot[:,6+i], color='b', linestyle='solid', linewidth=2)
    axes[i, 1].plot(t, qdot[:,11+i], color='b', linestyle='solid', linewidth=2)
    axes[i, 0].plot(t, jvel_cmd[:,i], color='r', linestyle='dashed', linewidth=4)
    axes[i, 1].plot(t, jvel_cmd[:,i+5], color='r', linestyle='dashed', linewidth=4)
    axes[i, 0].plot(t, jvel_max[i] * np.ones_like(qdot[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    axes[i, 0].plot(t, -jvel_max[i] * np.ones_like(qdot[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    axes[i, 1].plot(t, jvel_max[i+5] * np.ones_like(qdot[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    axes[i, 1].plot(t, -jvel_max[i+5] * np.ones_like(qdot[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    axes[i, 0].grid(True)
    axes[i, 1].grid(True)
    plot_phase(axes[i,0])
    plot_phase(axes[i,1])
fig.suptitle("Joint Vel")

fig, axes = plt.subplots(5, 2)
for i in range(5):
    axes[i,0].plot(t, trq_cmd[:,i], color='r', linestyle='solid', linewidth=3)
    axes[i,1].plot(t, trq_cmd[:,i+5], color='r', linestyle='solid', linewidth=3)
    axes[i,0].plot(t, jtrq_max[i] * np.ones_like(trq_cmd[:,i]), color='k', linestyle='solid', linewidth=3 )
    axes[i,0].plot(t, -jtrq_max[i] * np.ones_like(trq_cmd[:,i]), color='k', linestyle='solid', linewidth=3 )
    axes[i,1].plot(t, jtrq_max[i+5] * np.ones_like(trq_cmd[:,i]), color='k', linestyle='solid', linewidth=3 )
    axes[i,1].plot(t, -jtrq_max[i+5] * np.ones_like(trq_cmd[:,i]), color='k', linestyle='solid', linewidth=3 )
    axes[i,0].grid(True)
    axes[i,1].grid(True)
    plot_phase(axes[i,0])
    plot_phase(axes[i,1])
fig.suptitle("Joint Trq Des")

fig, axes = plt.subplots(6, 3)
for i in range(6):
    axes[i,0].plot(t, qddot_des[:,i], color='r', linestyle='solid', linewidth=3)
    plot_phase(axes[i,0])
    axes[i,0].grid(True)
for i in range(5):
    axes[i,1].plot(t, qddot_des[:,i+6], color='r', linestyle='solid', linewidth=3)
    axes[i,2].plot(t, qddot_des[:,i+11], color='r', linestyle='solid', linewidth=3)
    axes[i,1].grid(True)
    axes[i,2].grid(True)
    plot_phase(axes[i,0])
    plot_phase(axes[i,1])
fig.suptitle("Joint Acc Des")

plt.show()
