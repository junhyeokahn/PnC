import os
import sys

import pickle

import meshcat
import numpy as np
import time

# Robot model libraries
from pinocchio.visualize import MeshcatVisualizer
import pinocchio as pin

# Python-Meshcat
from meshcat.animation import Animation
from plot import meshcat_utils as vis_tools

cwd = os.getcwd()
sys.path.append(cwd)

# variables to load/display on Meshcat
exp_time = []
joint_positions = []
com_position_des = []
com_position = []
com_projection = []
base_joint_quat = []
lfoot_position = []
rfoot_position = []
lfoot_orientation = []
rfoot_orientation = []
lfoot_grf = []
rfoot_grf = []

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
viz.loadViewerModel(rootNodeName="draco")
vis_q = pin.neutral(model)

# add other visualizations to viewer
com_des_viz, com_des_model = vis_tools.add_sphere(viz.viewer, "com_des", color=[0., 0., 1., 0.5])
com_des_viz_q = pin.neutral(com_des_model)

com_viz, com_model = vis_tools.add_sphere(viz.viewer, "com", color=[1., 0., 0., 0.5])
com_viz_q = pin.neutral(com_model)

com_proj_viz, com_proj_model = vis_tools.add_sphere(viz.viewer, "com_proj", color=[0., 0., 1., 0.3])
com_proj_viz_q = pin.neutral(com_proj_model)

# add arrows visualizers to viewer
arrow_viz = meshcat.Visualizer(window=viz.viewer.window)
vis_tools.add_arrow(arrow_viz, "grf_lf", color=[0, 0, 1])
vis_tools.add_arrow(arrow_viz, "grf_rf", color=[1, 0, 0])

# load pkl data
with open('experiment_data/pnc.pkl', 'rb') as file:
    while True:
        try:
            d = pickle.load(file)
            exp_time.append(d['time'])
            lfoot_position.append(d['task_lfoot_lin_pos'])
            rfoot_position.append(d['task_rfoot_lin_pos'])
            lfoot_orientation.append(d['task_lfoot_ori_pos'])
            rfoot_orientation.append(d['task_rfoot_ori_pos'])
            lfoot_grf.append(d['cmd_lfoot_rf'])
            rfoot_grf.append(d['cmd_rfoot_rf'])
            joint_positions.append(d['base_joint_position'] +
                                   [d['base_joint_quat'][1], d['base_joint_quat'][2],
                                    d['base_joint_quat'][3], d['base_joint_quat'][0]] +
                                   d['joint_positions'])
            com_position_des.append(d['task_com_pos_des'])
            com_position.append(d['task_com_pos'])
            base_joint_quat.append(d['base_joint_quat'])
        except EOFError:
            break

# replay data and create animation
save_freq = 50          # hertz
anim = Animation(default_framerate=800 / save_freq)
frame_index = 0
for ti in range(len(exp_time)):
    viz.display(np.array(joint_positions[ti]))

    # plot misc parameters
    com_des_viz_q[0] = com_position_des[ti][0]
    com_des_viz_q[1] = com_position_des[ti][1]
    com_des_viz_q[2] = com_position_des[ti][2]
    com_des_viz.display(com_des_viz_q)

    com_proj_viz_q[0] = com_position_des[ti][0]
    com_proj_viz_q[1] = com_position_des[ti][1]
    com_proj_viz.display(com_proj_viz_q)

    com_viz_q[0] = com_position[ti][0]
    com_viz_q[1] = com_position[ti][1]
    com_viz_q[2] = com_position[ti][2]
    com_viz.display(com_viz_q)

    # plot GRFs
    vis_tools.grf_display(arrow_viz["grf_lf"], lfoot_position[ti], lfoot_orientation[ti], lfoot_grf[ti])
    vis_tools.grf_display(arrow_viz["grf_rf"], rfoot_position[ti], rfoot_orientation[ti], rfoot_grf[ti])

    # make animation
    with anim.at_frame(viz.viewer, frame_index) as frame:
        vis_tools.grf_display(frame["grf_lf"], lfoot_position[ti], lfoot_orientation[ti], lfoot_grf[ti])
        vis_tools.grf_display(frame["grf_rf"], rfoot_position[ti], rfoot_orientation[ti], rfoot_grf[ti])

        # save other visualizations at current frame
        vis_tools.display_visualizer_frames(viz, frame)                  # robot
        vis_tools.display_visualizer_frames(com_proj_viz, frame)         # projected CoM
        vis_tools.display_visualizer_frames(com_des_viz, frame)          # desired CoM position
        vis_tools.display_visualizer_frames(com_viz, frame)              # actual CoM position


    frame_index = frame_index + 1

print("Experiment initial time: ", exp_time[0])
print("Experiment final time: ", exp_time[-1])
viz.viewer.set_animation(anim, play=False)

