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
    arrow_shaft = g.Cylinder(0.1, 0.01)
    arrow_head = g.Cylinder(0.04, 0.04, radiusTop=0.001, radiusBottom=0.04)
    material = meshcat.geometry.MeshPhongMaterial()
    material.color = int(meshColor[0] * 255) * 256**2 + int(meshColor[1] * 255) * 256 + int(meshColor[2] * 255)

    meschat_visualizer[obj_name].set_object(arrow_shaft, material)
    meschat_visualizer[obj_name]["head"].set_object(arrow_head, material)


def set_grf_default_position(meschat_visualizer, foot_position):
    arrow_height = np.array([0, 0, 0.05])
    arrow_head_offset = np.array([0, arrow_height[2]+0.02/2, 0.0])
    T_rot = tf.rotation_matrix(np.pi/2, [1, 0, 0])
    T_trans = tf.translation_matrix(foot_position + arrow_height)
    T_trans_arrow_head = tf.translation_matrix(arrow_head_offset)

    # first translate, then rotate
    T = tf.concatenate_matrices(T_trans, T_rot)
    meschat_visualizer.set_transform(T)
    meschat_visualizer["head"].set_transform(T_trans_arrow_head)

def get_rpy_from_world_to(foot_grf):
    foot_grf_normalized = foot_grf[3:] / np.linalg.norm(foot_grf[3:])
    pitch = -np.arcsin(foot_grf_normalized[1])
    roll = np.arctan2(foot_grf_normalized[0], foot_grf_normalized[2])
    return np.array([roll, pitch, 0.])

def grf_display(meshcat_visualizer, foot_pos, foot_ori, foot_grf):
    # scale length
    scale = foot_grf[5] / 100.      # 200 is about half weight
    S = tf.identity_matrix()
    S[1, 1] = scale     # y-axis corresponds to height (i.e., length) of cylinder

    # translate and rotate GRF vectors
    arrow_height = np.array([0, 0, (0.1*scale)/2.])
    arrow_head_offset = np.array([0, 0.1/2, 0.0])
    # arrow_head_offset = np.array([0, arrow_height[2]+0.02/2, 0.0])
    T_arrow_vertical = tf.rotation_matrix(np.pi/2, [1, 0, 0])
    T_foot_rot = tf.euler_matrix(foot_ori[0], foot_ori[1], foot_ori[2])
    grf_ori = get_rpy_from_world_to(foot_grf)
    T_grf_ori = tf.euler_matrix(grf_ori[0], grf_ori[1], grf_ori[2])
    T_trans = tf.translation_matrix(foot_pos + arrow_height)
    T_trans_arrow_head = tf.translation_matrix(arrow_head_offset)

    # first translate, then rotate, and scale
    # T = tf.concatenate_matrices(T_trans, T_foot_rot, T_arrow_vertical, S)
    T = tf.concatenate_matrices(T_trans, T_grf_ori, T_arrow_vertical, S)
    meshcat_visualizer.set_transform(T)
    meshcat_visualizer["head"].set_transform(T_trans_arrow_head)


def display_visualizer_frames(meshcat_visualizer):
    for visual in meshcat_visualizer.visual_model.geometryObjects:
        # Get mesh pose.
        M = meshcat_visualizer.visual_data.oMg[meshcat_visualizer.visual_model.getGeometryId(visual.name)]
        # Manage scaling
        scale = np.asarray(visual.meshScale).flatten()
        S = np.diag(np.concatenate((scale, [1.0])))
        T = np.array(M.homogeneous).dot(S)
        # Update viewer configuration.
        frame[meshcat_visualizer.getViewerNodeName(visual, pin.GeometryType.VISUAL)].set_transform(T)


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

# replay data
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
    grf_display(viz.viewer["grf_lf"], lfoot_position[ti], lfoot_orientation[ti], lfoot_grf[ti])
    grf_display(viz.viewer["grf_rf"], rfoot_position[ti], rfoot_orientation[ti], rfoot_grf[ti])

    # make animation
    with anim.at_frame(viz.viewer, frame_index) as frame:
        grf_display(frame["grf_lf"], lfoot_position[ti], lfoot_orientation[ti], lfoot_grf[ti])
        grf_display(frame["grf_rf"], rfoot_position[ti], rfoot_orientation[ti], rfoot_grf[ti])

        # save other visualizations at current frame
        display_visualizer_frames(viz)                  # robot
        display_visualizer_frames(com_proj_viz)         # projected CoM
        display_visualizer_frames(com_des_viz)          # desired CoM position
        display_visualizer_frames(com_viz)              # actual CoM position


    frame_index = frame_index + 1

print("Experiment initial time: ", exp_time[0])
print("Experiment final time: ", exp_time[-1])
viz.viewer.set_animation(anim)

