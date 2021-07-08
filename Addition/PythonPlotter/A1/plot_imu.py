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

st_idx = 5
end_idx = len(t) - 2000
t = t[st_idx:end_idx]

data_phse = np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))[st_idx:end_idx]

phseChange = []
for i in range(0,len(t)-1):
        if data_phse[i] != data_phse[i+1]:
            phseChange.append(i)
        else:
            pass

imu_acc = np.genfromtxt(file_path+'imu_acc.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
imu_ang_vel = np.genfromtxt(file_path+'imu_ang_vel.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
imu_rpy = np.genfromtxt(file_path+'imu_rpy.txt', delimiter=None, dtype=(float))[st_idx:end_idx]

print("t length",len(t))
print("rpy length",len(imu_rpy))
print("ang_vel length",len(imu_ang_vel))
print("acc length",len(imu_acc))

## -----------------------------------------------------------------------------
## Plot Cmd
## -----------------------------------------------------------------------------
def plot_phase(ax):
    for j in phseChange:
        ax.axvline(x=t[j],color='indigo',linestyle='-')
        ax.text(t[j],ax.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

fig1, axes1 = plt.subplots(1, 3)
axes1[0].plot(t, imu_acc[:,0], color='b', linestyle='dashed', linewidth=2)
axes1[0].set_ylabel('linear acc x')
axes1[1].plot(t, imu_acc[:,1], color='b', linestyle='dashed', linewidth=2)
axes1[1].set_ylabel('linear acc y')
axes1[2].plot(t, imu_acc[:,2], color='b', linestyle='dashed', linewidth=2)
axes1[2].set_ylabel('linear acc z')

for i in range(3):   
    axes1[i].grid(True)
    plot_phase(axes1[i])
# plt.show()

fig2, axes2 = plt.subplots(1, 3)
axes2[0].plot(t, imu_ang_vel[:,0], color='b', linestyle='dashed', linewidth=2)
axes2[0].set_ylabel('linear gyro x')
axes2[1].plot(t, imu_ang_vel[:,1], color='b', linestyle='dashed', linewidth=2)
axes2[1].set_ylabel('linear gyro y')
axes2[2].plot(t, imu_ang_vel[:,2], color='b', linestyle='dashed', linewidth=2)
axes2[2].set_ylabel('linear gyro z')

for i in range(3):   
    axes2[i].grid(True)
    plot_phase(axes2[i])
# plt.show()


fig3, axes3 = plt.subplots(1, 3)
axes3[0].plot(t, imu_rpy[:,0], color='b', linestyle='dashed', linewidth=2)
axes3[0].set_ylabel('roll')
axes3[1].plot(t, imu_rpy[:,1], color='b', linestyle='dashed', linewidth=2)
axes3[1].set_ylabel('pitch')
axes3[2].plot(t, imu_rpy[:,2], color='b', linestyle='dashed', linewidth=2)
axes3[2].set_ylabel('yaw')

for i in range(3):   
    axes3[i].grid(True)
    plot_phase(axes3[i])
plt.show()