import numpy as np

import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

data = []
data.append(np.genfromtxt("experiment_data/4_xddot_res.txt"))
data.append(np.genfromtxt("experiment_data/5_xddot_res.txt"))
data.append(np.genfromtxt("experiment_data/6_xddot_res.txt"))
data.append(np.genfromtxt("experiment_data/7_xddot_res.txt"))

fig, axes = plt.subplots(3, 4)

for col_id, d in enumerate(data):
    for row_id in range(3):
        axes[row_id, col_id].plot(d[:, row_id])
        axes[row_id, col_id].grid(True)

plt.show()
