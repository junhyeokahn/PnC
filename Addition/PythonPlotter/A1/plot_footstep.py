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
end_idx = len(t)
t = t[st_idx:end_idx]

flfoot_landing = np.genfromtxt(file_path+'flfoot_landing.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
frfoot_landing = np.genfromtxt(file_path+'frfoot_landing.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
rlfoot_landing = np.genfromtxt(file_path+'rlfoot_landing.txt', delimiter=None, dtype=(float))[st_idx:end_idx]
rrfoot_landing = np.genfromtxt(file_path+'rrfoot_landing.txt', delimiter=None, dtype=(float))[st_idx:end_idx]

fl_xs = flfoot_landing[:,0]
fl_ys = -flfoot_landing[:,1]
# print("fl_xs = ",fl_xs)
# print("fl_ys = ",fl_ys)

fr_xs = frfoot_landing[:,0]
fr_ys = -frfoot_landing[:,1]

rl_xs = rlfoot_landing[:,0]
rl_ys = -rlfoot_landing[:,1]

rr_xs = rrfoot_landing[:,0]
rr_ys = -rrfoot_landing[:,1]

plt.scatter(fl_ys, fl_xs, c='r', label='FL')
plt.scatter(fr_ys, fr_xs, c='b', label='FR')
plt.scatter(rl_ys, rl_xs, c='g', label='RL')
plt.scatter(rr_ys, rr_xs, c='y', label='RR')
plt.xlabel("y Position [m]")
plt.ylabel("x Position [m]")
plt.title("Footstep Planner")
plt.xlim([-0.3,0.3])
plt.ylim([-0.3,0.3])
plt.grid(True)
plt.legend()
plt.show()