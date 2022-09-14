import os
import sys

import pickle

import numpy as np
import time

# Robot model libraries
from pinocchio.visualize import MeshcatVisualizer
import pinocchio as pin
import ipdb

cwd = os.getcwd()
sys.path.append(cwd)

# variables to load/display on Meshcat
exp_time = []
joint_positions = []
base_position = []
base_orientation = []
base_orientation_shift = []

# Create Robot for Meshcat Visualization
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    cwd + "/robot_model/draco/draco.urdf", cwd + "/robot_model/draco",
    pin.JointModelFreeFlyer())
viz = MeshcatVisualizer(model, collision_model, visual_model)
try:
    viz.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)
viz.loadViewerModel()
vis_q = pin.neutral(model)

with open(cwd + '/experiment_data/pnc.pkl', 'rb') as file:
    while True:
        try:
            d = pickle.load(file)
            exp_time.append(d['time'])
            # base_orientation.append(d['base_joint_quat'])
            # base_position.append(d['base_joint_position'])
            # print(base_orientation_shift)
            # print(d['base_joint_quat'])
            # joint_positions.append(d['base_joint_position'] +
            # base_orientation_shift +
            # d['joint_positions'])
            joint_positions.append(d['base_joint_position'] + [
                d['base_joint_quat'][1], d['base_joint_quat'][2],
                d['base_joint_quat'][3], d['base_joint_quat'][0]
            ] + d['joint_positions'])

        except EOFError:
            break

# display poses
# note: the sleep time should ideally be: save_freq / loop_rate (e.g., 50/800) but this doesn't
# take into account compute time of this PC, so a somewhat faster value should be used
save_freq = 20  # hertz
for ti in range(len(exp_time)):
    viz.display(np.array(joint_positions[ti]))
    time.sleep(1.0 / save_freq)
