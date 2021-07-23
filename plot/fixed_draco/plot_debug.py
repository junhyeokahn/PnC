import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
import pickle

import numpy as np
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from plot.helper import plot_task, plot_weights, plot_rf_z_max, plot_rf, plot_vector_traj

file_path = os.getcwd() + '/experiment_data/'
debug_topic = np.genfromtxt(file_path + 'tmp.txt', delimiter=None, dtype=float)

plot_vector_traj(np.arange(debug_topic.shape[0]), debug_topic, None, None,
                 None)

plt.show()
