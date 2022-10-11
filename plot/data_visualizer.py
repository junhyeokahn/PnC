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
icp_des, icp_act = [], []

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

icp_model, icp_collision_model, icp_visual_model = pin.buildModelsFromUrdf(
    "robot_model/ground/sphere.urdf", "robot_model/ground",
    pin.JointModelFreeFlyer())
icp_viz = MeshcatVisualizer(icp_model, icp_collision_model, icp_visual_model)
icp_viz.initViewer(viz.viewer)
icp_viz.loadViewerModel(rootNodeName="icp", color=[0., 0., 1., 0.5])
icp_viz_q = pin.neutral(icp_model)

icp_des_model, icp_des_collision_model, icp_des_visual_model = pin.buildModelsFromUrdf(
    "robot_model/ground/sphere.urdf", "robot_model/ground",
    pin.JointModelFreeFlyer())
icp_des_viz = MeshcatVisualizer(icp_des_model, icp_des_collision_model,
                                icp_des_visual_model)
icp_des_viz.initViewer(viz.viewer)
icp_des_viz.loadViewerModel(rootNodeName="icp_des", color=[1., 0., 0., 0.5])
icp_des_viz_q = pin.neutral(icp_des_model)

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

            icp_des.append(d['icp_des'])
            icp_act.append(d['icp'])

        except EOFError:
            break

# display poses
# note: the sleep time should ideally be: save_freq / loop_rate (e.g., 50/800) but this doesn't
# take into account compute time of this PC, so a somewhat faster value should be used
save_freq = 20  # hertz
for ti in range(len(exp_time)):
    viz.display(np.array(joint_positions[ti]))
    icp_des_viz_q[0] = icp_des[ti][0]
    icp_des_viz_q[1] = icp_des[ti][1]
    icp_viz_q[0] = icp_act[ti][0]
    icp_viz_q[1] = icp_act[ti][1]

    icp_viz.display(icp_viz_q)
    icp_des_viz.display(icp_des_viz_q)
    time.sleep(1.0 / save_freq)
