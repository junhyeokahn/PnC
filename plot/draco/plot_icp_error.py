import numpy as np
import copy
import pickle

import utils.python_utils.digital_filters as filt

from plot.helper import *

matplotlib.use('TkAgg')

save_data = 50
st_idx = 10

time = []
phase = []
icp = []
icp_des = []
icp_dot = []
icp_dot_des = []
data_icp_err_integrator = []
data_icp_err_leaky_integral = []

with open('experiment_data/pnc.pkl', 'rb') as file:
    while True:
        try:
            d = pickle.load(file)
            time.append(d['time'])
            phase.append(d['phase'])
            icp.append(d['icp'])
            icp_des.append(d['icp_des'])
            icp_dot.append(d['icp_dot'])
            icp_dot_des.append(d['icp_dot_des'])
            data_icp_err_integrator.append(d['icp_err_integrator'])
            data_icp_err_leaky_integral .append(d['icp_err_leaky_integral'])
        except EOFError:
            break
#
# stack
#
time = np.array(time)[st_idx:]
phase = np.array(phase)[st_idx:]
icp = np.stack(icp, axis=0)[st_idx:, :]
icp_des = np.stack(icp_des, axis=0)[st_idx:, :]
icp_dot = np.stack(icp_dot, axis=0)[st_idx:, :]
icp_dot_des = np.stack(icp_dot_des, axis=0)[st_idx:, :]
data_icp_err_integrator = np.stack(data_icp_err_integrator, axis=0)[st_idx:, :]
data_icp_err_leaky_integral = np.stack(data_icp_err_leaky_integral, axis=0)[st_idx:, :]

#
# misc
#
dt = 0.00125
time_constant = 0.5
init_value = icp[st_idx-1, :] - icp_des[st_idx-1, :]
icp_err_lim = np.array([0.03, 0.03])
exp_filt = filt.ExponentialMovingAverage(save_data * dt, time_constant, init_value, -icp_err_lim, icp_err_lim)

leak_rate = 0.5


#
# process for analysis
#
icp_err = icp_des - icp
icp_dot_err = icp_dot_des - icp_dot
icp_integrator_err = np.zeros((icp_err.shape))
for i in range(time.shape[0]):
    exp_filt.input(icp_err[i, :])
    icp_integrator_err[i, :] = exp_filt.output()

icp_err_integral = np.zeros((icp_err.shape))
for i in range(1, time.shape[0]):
    leaky_err = icp_err[i, :] * save_data * dt + leak_rate * icp_err_integral[i-1, :]
    icp_err_integral[i, :] = np.clip(leaky_err, -icp_err_lim, icp_err_lim)

#
# plot
#
plot_task(time, np.zeros((icp_err.shape)), icp_err,
          np.zeros((icp_dot_err.shape)), icp_dot_err, phase, 'icp errors')

# current integrator
icp_pid_errors = np.concatenate((icp_err, icp_integrator_err, icp_dot_err), axis=1)

# proposed (leaky) integrator
icp_leaky_pid_errors = np.concatenate((icp_err, icp_err_integral, icp_dot_err), axis=1)

# data icp integrator error
# py_c_icp_exp_integral_errors = np.zeros((icp_err.shape[0], 4))
# py_c_icp_exp_integral_errors[:, :2] = copy.deepcopy(icp_integrator_err)
# py_c_icp_exp_integral_errors[:, 2:] = copy.deepcopy(data_icp_err_integrator)

# py_c_icp_leaky_integral_errors = np.zeros((icp_err.shape[0], 4))
# py_c_icp_leaky_integral_errors[:, :2] = copy.deepcopy(icp_err_integral)
# py_c_icp_leaky_integral_errors[:, 2:] = copy.deepcopy(data_icp_err_leaky_integral)

# axes = plot_vector_traj(time, icp_pid_errors[:, [0, 2, 4]], phase, ['P error', 'I error', 'D error'], 'k', 'ICP-x PID errors')
# plot_vector_traj(time, icp_leaky_pid_errors[:, [0, 2, 4]], phase, ['P error', 'I error', 'D error'], 'b', axes=axes)
# axes = plot_vector_traj(time, icp_pid_errors[:, [1, 3, 5]], phase, ['P error', 'I error', 'D error'], 'k', 'ICP-y PID errors')
# plot_vector_traj(time, icp_leaky_pid_errors[:, [1, 3, 5]], phase, ['P error', 'I error', 'D error'], 'b', axes=axes)

# compare python and c++ integral error implementations
# axes = plot_vector_traj(time, py_c_icp_exp_integral_errors[:, [0, 1]], phase, ['P error', 'I error', 'D error'], 'k', 'ICP PID (exp) err')
# plot_vector_traj(time, py_c_icp_exp_integral_errors[:, [2, 3]], phase, ['P error', 'I error', 'D error'], 'b', axes=axes)
# axes = plot_vector_traj(time, py_c_icp_leaky_integral_errors[:, [0, 1]], phase, ['P error', 'I error', 'D error'], 'k', 'ICP PID (leaky) err')
# plot_vector_traj(time, py_c_icp_leaky_integral_errors[:, [2, 3]], phase, ['P error', 'I error', 'D error'], 'b', axes=axes)

# compare data from different integral error implementations
axes = plot_vector_traj(time, icp_err, phase, ['x error', 'y error'], 'k', suptitle='ICP errors', legend='error')
axes = plot_vector_traj(time, data_icp_err_integrator, phase, ['x error', 'y error'], 'b', legend='exp int', axes=axes)
plot_vector_traj(time, 250.*data_icp_err_leaky_integral, phase, ['x error', 'y error'], 'g', legend='250*leaky int', axes=axes)

plt.show()
