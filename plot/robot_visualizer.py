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
import meshcat.geometry as g
import meshcat.transformations as tf
from meshcat.animation import Animation

cwd = os.getcwd()
sys.path.append(cwd)


def add_arrow(meschat_visualizer, obj_name, meshColor=[1, 0, 0]):
    obj = g.Cylinder(0.1, 0.02)
    material = meshcat.geometry.MeshPhongMaterial()
    material.color = int(meshColor[0] * 255) * 256**2 + int(meshColor[1] * 255) * 256 + int(meshColor[2] * 255)

    meschat_visualizer[obj_name].set_object(obj, material)

    return obj


def set_grf_default_position(meschat_visualizer, foot_position):
    arrow_height = [0, 0, 0.05]
    T_rot = tf.rotation_matrix(np.pi/2, [1, 0, 0])
    T_trans = tf.translation_matrix(foot_position + arrow_height)

    # first translate, then rotate
    T = tf.concatenate_matrices(T_trans, T_rot)
    meschat_visualizer.set_transform(T)


def grf_display(meshcat_visualizer, foot_pos, foot_ori, foot_grf):
    arrow_height = np.array([0, 0, 0.05])
    T_arrow_rot = tf.rotation_matrix(np.pi/2, [1, 0, 0])
    T_foot_rot = tf.euler_matrix(foot_ori[0], foot_ori[1], foot_ori[2])
    T_trans = tf.translation_matrix(foot_pos + arrow_height)
    scale = foot_grf[5] / 200.      # 200 is about half weight
    S = tf.scale_matrix(scale)

    # first translate, then rotate, and scale
    T = tf.concatenate_matrices(T_trans, T_foot_rot, T_arrow_rot, S)
    meshcat_visualizer.set_transform(T)


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
# viz = meshcat.Visualizer(model, collision_model, visual_model)
try:
    viz.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)
# obj = loadViewerModel(cwd + "/robot_model/draco")
# viz.set_object(obj)
viz.loadViewerModel()
vis_q = pin.neutral(model)

com_des_model, com_des_collision_model, com_des_visual_model = pin.buildModelsFromUrdf(
    "robot_model/ground/sphere.urdf", "robot_model/ground",
    pin.JointModelFreeFlyer())
com_des_viz = MeshcatVisualizer(com_des_model, com_des_collision_model, com_des_visual_model)
com_des_viz.initViewer(viz.viewer)
com_des_viz.loadViewerModel(rootNodeName="com_des", color=[0., 0., 1., 0.5])
com_des_viz_q = pin.neutral(com_des_model)

com_model, com_collision_model, com_visual_model = pin.buildModelsFromUrdf(
    "robot_model/ground/sphere.urdf", "robot_model/ground",
    pin.JointModelFreeFlyer())
com_viz = MeshcatVisualizer(com_model, com_collision_model, com_visual_model)
com_viz.initViewer(viz.viewer)
com_viz.loadViewerModel(rootNodeName="com", color=[1., 0., 0., 0.5])
com_viz_q = pin.neutral(com_model)


com_proj_model, com_proj_collision_model, com_proj_visual_model = pin.buildModelsFromUrdf(
    "robot_model/ground/sphere.urdf", "robot_model/ground",
    pin.JointModelFreeFlyer())
com_proj_viz = MeshcatVisualizer(com_proj_model, com_proj_collision_model,
                                 com_proj_visual_model)
com_proj_viz.initViewer(viz.viewer)
com_proj_viz.loadViewerModel(rootNodeName="com_proj", color=[0., 0., 1., 0.3])
com_proj_viz_q = pin.neutral(com_proj_model)

# create arrows
add_arrow(viz.viewer, "grf_lf", meshColor=[0, 0, 1])
add_arrow(viz.viewer, "grf_rf", meshColor=[1, 0, 0])
# viz.viewer["grf_arrow"].set_object(g.StlMeshGeometry.from_file(cwd+"/robot_model/ground/arrow.urdf"))

# transform arrows
lfoot_pos = np.array([0, 0.11, 0])
rfoot_pos = np.array([0, -0.11, 0])
set_grf_default_position(viz.viewer["grf_lf"], lfoot_pos)
set_grf_default_position(viz.viewer["grf_rf"], rfoot_pos)
# viz.viewer["grf_rf"].set_transform(tf.translation_matrix([0, 1, 0]))

# arrow_model, arrow_collision_model, arrow_visual_model = pin.buildModelsFromUrdf(
#     "robot_model/ground/arrow.urdf", "robot_model/ground",
#     pin.JointModelFreeFlyer())
# arrow_viz = MeshcatVisualizer(arrow_model, arrow_collision_model, arrow_visual_model)
# arrow_viz.initViewer(viz.viewer)
# arrow_viz.loadViewerModel(rootNodeName="arrow", color=[0.5, 0.5, 0.5, 0.5])
# arrow_viz_q = pin.neutral(arrow_model)

with open('experiment_data/pnc.pkl', 'rb') as file:
    while True:
        try:
            d = pickle.load(file)
            exp_time.append(d['time'])
            # base_orientation.append(d['base_joint_quat'])
            # base_position.append(d['base_joint_pos'])
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

anim = Animation()

# display poses
# note: the sleep time should ideally be: save_freq / loop_rate (e.g., 50/800) but this doesn't
# take into account compute time of this PC, so a somewhat faster value should be used
save_freq = 40          # hertz
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
    grf_display(viz.viewer["grf_lf"], lfoot_position[ti], lfoot_orientation[ti], lfoot_grf[ti])
    grf_display(viz.viewer["grf_rf"], rfoot_position[ti], rfoot_orientation[ti], rfoot_grf[ti])

    # make animation
    # with anim.at_frame(viz.viewer, 0) as frame:
    #     frame["grf_lf"].set

    print("experiment time: ", exp_time[ti])
    time.sleep(1.0 / save_freq)



