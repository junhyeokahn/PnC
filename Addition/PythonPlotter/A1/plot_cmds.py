import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

## -----------------------------------------------------------------------------
## Read Data
## -----------------------------------------------------------------------------
file_path = os.getcwd() + "/../../../ExperimentData/"

t = np.genfromtxt(file_path+'running_time.txt', delimiter='\n', dtype=(float))

st_idx = 1
end_idx = len(t)
t = t[st_idx:end_idx]

jpos_cmd = np.genfromtxt(file_path+"jpos_des.txt", delimiter=None, dtype=(float))[st_idx:end_idx]
jvel_cmd = np.genfromtxt(file_path+"jvel_des.txt", delimiter=None, dtype=(float))[st_idx:end_idx]
trq_cmd = np.genfromtxt(file_path+"command_torque.txt", delimiter=None, dtype=(float))[st_idx:end_idx]
q = np.genfromtxt(file_path+'config.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
qdot = np.genfromtxt(file_path+'qdot.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
qddot_des = np.genfromtxt(file_path+'qddot_des.txt', delimiter=None, dtype=(float))[st_idx:end_idx]

jpos_min = np.array([-0.8 , -1.04 , -2.70 , -0.8 , -1.04 , -2.70 , -0.8 , -1.04 , -2.70 , -0.8 , -1.04 , -2.70])
jpos_max = np.array([0.8  ,  4.19 , -0.91 , 0.8  , 4.19  , -0.91 , 0.8  , 4.19  , -0.91 , 0.8  , 4.19  , -0.91 ])
jvel_max = np.array([52.4   , 28.6   , 28.6   , 52.4   , 28.6   , 28.6   , 52.4   , 28.6   , 28.6  , 52.4  ,  28.6  ,  28.6])
jtrq_max = np.array([20.   , 55.   , 55.   , 20.   , 55.   , 55.   , 20.   , 55.   , 55.   , 20.  , 55.  , 55.])

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

fig, axes = plt.subplots(3, 4)
for i in range(3):
    # actual q for the 4 legs
    axes[i, 0].plot(t, q[:,6+i], color='b', linestyle='solid', linewidth=2)
    axes[i, 1].plot(t, q[:,9+i], color='b', linestyle='solid', linewidth=2)
    axes[i, 2].plot(t, q[:,12+i], color='b', linestyle='solid', linewidth=2)
    axes[i, 3].plot(t, q[:,15+i], color='b', linestyle='solid', linewidth=2)
    # jpos cmd for 4 legs
    axes[i, 0].plot(t, jpos_cmd[:,i], color='r', linestyle='dashed', linewidth=4)
    axes[i, 1].plot(t, jpos_cmd[:,i+3], color='r', linestyle='dashed', linewidth=4)
    axes[i, 2].plot(t, jpos_cmd[:,i+6], color='r', linestyle='dashed', linewidth=4)
    axes[i, 3].plot(t, jpos_cmd[:,i+9], color='r', linestyle='dashed', linewidth=4)
    axes[i, 0].plot(t, jpos_max[i] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    axes[i, 0].plot(t, jpos_min[i] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    axes[i, 1].plot(t, jpos_max[i+3] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    axes[i, 1].plot(t, jpos_min[i+3] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    axes[i, 2].plot(t, jpos_max[i+6] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    axes[i, 2].plot(t, jpos_min[i+6] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    axes[i, 3].plot(t, jpos_max[i+9] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    axes[i, 3].plot(t, jpos_min[i+9] * np.ones_like(q[:,6+i]), color='k', linestyle='solid', linewidth=3 )
    axes[i, 0].grid(True)
    axes[i, 1].grid(True)
    axes[i, 2].grid(True)
    axes[i, 3].grid(True)
    # plot_phase(axes[i,0])
    # plot_phase(axes[i,1])
fig.suptitle("Joint Pos")


fig, axes = plt.subplots(3, 4)
for i in range(3):
    # actual qdot for the 4 legs
    axes[i, 0].plot(t, qdot[:,6+i], color='b', linestyle='solid', linewidth=2)
    axes[i, 1].plot(t, qdot[:,9+i], color='b', linestyle='solid', linewidth=2)
    axes[i, 2].plot(t, qdot[:,12+i], color='b', linestyle='solid', linewidth=2)
    axes[i, 3].plot(t, qdot[:,15+i], color='b', linestyle='solid', linewidth=2)
    # jvel cmd for 4 legs
    axes[i, 0].plot(t, jvel_cmd[:,i], color='r', linestyle='dashed', linewidth=4)
    axes[i, 1].plot(t, jvel_cmd[:,i+3], color='r', linestyle='dashed', linewidth=4)
    axes[i, 2].plot(t, jvel_cmd[:,i+6], color='r', linestyle='dashed', linewidth=4)
    axes[i, 3].plot(t, jvel_cmd[:,i+9], color='r', linestyle='dashed', linewidth=4)
    axes[i, 0].grid(True)
    axes[i, 1].grid(True)
    axes[i, 2].grid(True)
    axes[i, 3].grid(True)
    fig.suptitle("Joint Vel")

fig, axes = plt.subplots(3, 4)
for i in range(3):
    # jvel cmd for 4 legs
    axes[i, 0].plot(t, trq_cmd[:,i], color='r', linestyle='dashed', linewidth=4)
    axes[i, 1].plot(t, trq_cmd[:,i+3], color='r', linestyle='dashed', linewidth=4)
    axes[i, 2].plot(t, trq_cmd[:,i+6], color='r', linestyle='dashed', linewidth=4)
    axes[i, 3].plot(t, trq_cmd[:,i+9], color='r', linestyle='dashed', linewidth=4)
 
    # axes[i,0].plot(t, jtrq_max[i] * np.ones_like(trq_cmd[:,i]), color='k', linestyle='solid', linewidth=3 )
    # axes[i,0].plot(t, -jtrq_max[i] * np.ones_like(trq_cmd[:,i]), color='k', linestyle='solid', linewidth=3 )
    # axes[i,1].plot(t, jtrq_max[i+5] * np.ones_like(trq_cmd[:,i]), color='k', linestyle='solid', linewidth=3 )
    # axes[i,1].plot(t, -jtrq_max[i+5] * np.ones_like(trq_cmd[:,i]), color='k', linestyle='solid', linewidth=3 )
    axes[i,0].grid(True)
    axes[i,1].grid(True)
    axes[i, 2].grid(True)
    axes[i, 3].grid(True)
    # plot_phase(axes[i,0])
    # plot_phase(axes[i,1])
fig.suptitle("Joint Trq Des")

plt.show()
