import numpy as np

from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as g
import meshcat.transformations as tf

import pinocchio as pin

violet = [1., 0., 1., 0.3]

def add_arrow(meshcat_visualizer, obj_name, color=[1, 0, 0], height=0.1):
    arrow_shaft = g.Cylinder(height, 0.01)
    arrow_head = g.Cylinder(0.04, 0.04, radiusTop=0.001, radiusBottom=0.04)
    material = g.MeshPhongMaterial()
    material.color = int(color[0] * 255) * 256**2 + int(color[1] * 255) * 256 + int(color[2] * 255)

    meshcat_visualizer[obj_name].set_object(arrow_shaft, material)
    meshcat_visualizer[obj_name]["head"].set_object(arrow_head, material)


def add_sphere(parent_visualizer, node_name="sphere",
               urdf_path="robot_model/ground/sphere.urdf",
               visuals_path="robot_model/ground", color=[0., 0., 1., 0.5]):
    sphere_model, sphere_collision_model, sphere_visual_model = pin.buildModelsFromUrdf(
        urdf_path, visuals_path, pin.JointModelFreeFlyer())
    sphere_viz = MeshcatVisualizer(sphere_model, sphere_collision_model, sphere_visual_model)
    sphere_viz.initViewer(parent_visualizer)
    sphere_viz.loadViewerModel(rootNodeName=node_name, color=color)

    return sphere_viz, sphere_model


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
    grf_ori = get_rpy_from_world_to(foot_grf)
    T_grf_ori = tf.euler_matrix(grf_ori[0], grf_ori[1], grf_ori[2])
    T_trans = tf.translation_matrix(foot_pos + arrow_height)
    T_trans_arrow_head = tf.translation_matrix(arrow_head_offset)

    # first translate, then rotate, and scale
    T = tf.concatenate_matrices(T_trans, T_grf_ori, T_arrow_vertical, S)
    meshcat_visualizer.set_transform(T)
    meshcat_visualizer["head"].set_transform(T_trans_arrow_head)


def display_visualizer_frames(meshcat_visualizer, frame):
    for visual in meshcat_visualizer.visual_model.geometryObjects:
        # Get mesh pose.
        M = meshcat_visualizer.visual_data.oMg[meshcat_visualizer.visual_model.getGeometryId(visual.name)]
        # Manage scaling
        scale = np.asarray(visual.meshScale).flatten()
        S = np.diag(np.concatenate((scale, [1.0])))
        T = np.array(M.homogeneous).dot(S)
        # Update viewer configuration.
        frame[meshcat_visualizer.getViewerNodeName(visual, pin.GeometryType.VISUAL)].set_transform(T)
