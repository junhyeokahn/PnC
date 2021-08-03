import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
import pickle

import numpy as np
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from plot.helper import plot_vector_traj

time = []
base_joint_quat = []

with open('experiment_data/pnc.pkl', 'rb') as file:
    while True:
        try:
            d = pickle.load(file)
            time.append(d['time'])
            base_joint_quat.append(d['base_joint_quat'])
        except EOFError:
            break

base_joint_quat = np.stack(base_joint_quat, axis=0)

plot_vector_traj(time, base_joint_quat, None, "base joint quat",
                 ['w', 'x', 'y', 'z'])

plt.show()
