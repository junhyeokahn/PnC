import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

figure_number = 0

file_path = os.getcwd() + "/../../../ExperimentDataCheck/"

## read files
data_r_vrp_pos = \
np.genfromtxt(file_path+'r_vrp.txt', delimiter=None, dtype=(float))
data_dcm_pos = \
np.genfromtxt(file_path+'dcm.txt', delimiter=None, dtype=(float))
data_dcm_pos_des = \
np.genfromtxt(file_path+'dcm_des.txt', delimiter=None, dtype=(float))
data_r_vrp_pos_des = \
np.genfromtxt(file_path+'r_vrp_des.txt', delimiter=None, dtype=(float))

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

## plot rvrp time
fig = plt.figure(figure_number)
fig.canvas.set_window_title('r_vrp pos')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_r_vrp_pos[st_idx:end_idx,i-1], "k-")
    plt.plot(data_x, data_r_vrp_pos_des[st_idx:end_idx,i-1], "r-")
    # plt.plot(data_x, data_dcm_pos[st_idx:end_idx,i-1], "k-")
    plt.grid(True)
    for j in phseChange:
        plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
plt.xlabel('time (sec)')

figure_number += 1

# Filter RVRP position. IF VRP desired is not greater than z_cutoff then we cannot make a valid rvrp x-y plot
z_cutoff = 0.5  
for i in range(len(data_x)):
    if (data_r_vrp_pos_des[i, 2] >= z_cutoff):
        print "cutoff found at : ", i, data_r_vrp_pos_des[i, 2]
        st_idx = i
        break

print st_idx 

## plot rvrp x-y
fig = plt.figure(figure_number)
fig.canvas.set_window_title('r_vrp x-y')
ax1 = plt.subplot(1, 1, 1)
plt.plot(data_dcm_pos[st_idx:end_idx,0], data_dcm_pos[st_idx:end_idx,1], "bo")
plt.plot(data_dcm_pos_des[st_idx:end_idx:10,0], data_dcm_pos_des[st_idx:end_idx:10,1], "co-")
plt.plot(data_r_vrp_pos[st_idx:end_idx,0], data_r_vrp_pos[st_idx:end_idx,1], "kx")
plt.plot(data_r_vrp_pos_des[st_idx:end_idx,0], data_r_vrp_pos_des[st_idx:end_idx,1], "r-")

plt.xlabel('r_vrp (x pos)')
plt.ylabel('r_vrp (y pos)')
plt.axis('square')

plt.show()
