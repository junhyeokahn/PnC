import zmq
import sys
import os

sys.path.append(os.getcwd() + '/build')
sys.path.append(os.getcwd() + 'plot')
sys.path.append(os.getcwd())
import time

import json
from ruamel.yaml import YAML
import numpy as np
from pinocchio.visualize import MeshcatVisualizer
import pinocchio as pin

from utils.python_utils import util

from plot.data_saver import DataSaver

from messages.draco_pb2 import *
from messages.draco_pybullet_sensors_pb2 import *

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--b_visualize", type=bool, default=False)
args = parser.parse_args()

with open("config/draco/pnc.yaml", 'r') as stream:
    config = YAML().load(stream)
    addr = config["ip_addr"]

context = zmq.Context()
pnc_socket = context.socket(zmq.SUB)
pnc_socket.connect(addr)
pnc_socket.setsockopt_string(zmq.SUBSCRIBE, "")

if not config["b_exp"]:
    gt_socket = context.socket(zmq.SUB)
    gt_socket.connect(
        "tcp://localhost:5558"
    )  # match to PUB address from sim, e.g., from pybullet_simulation.py
    gt_socket.setsockopt_string(zmq.SUBSCRIBE, "")
    gt_msg = bullet_gt_msg()  # messages that live only within python

pj_context = zmq.Context()
pj_socket = pj_context.socket(zmq.PUB)
pj_socket.bind("tcp://*:9872")

data_saver = DataSaver()

if args.b_visualize:
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        "robot_model/draco/draco.urdf", "robot_model/draco",
        pin.JointModelFreeFlyer())
    viz = MeshcatVisualizer(model, collision_model, visual_model)

    # add KF model
    model_kf, collision_model_kf, visual_model_kf = pin.buildModelsFromUrdf(
        "robot_model/draco/draco.urdf", "robot_model/draco",
        pin.JointModelFreeFlyer())
    viz_kf = MeshcatVisualizer(model_kf, collision_model_kf, visual_model_kf)
    try:
        viz.initViewer(open=True)
    except ImportError as err:
        print(
            "Error while initializing the viewer. It seems you should install python meshcat"
        )
        print(err)
        exit()
    viz.loadViewerModel()
    vis_q = pin.neutral(model)
    vis_q_kf = pin.neutral(model_kf)

    viz_kf.initViewer(viz.viewer)
    viz_kf.loadViewerModel(rootNodeName="kf_model", color=[0.2, 0.2, 0.2, 0.3])
    icp_model, icp_collision_model, icp_visual_model = pin.buildModelsFromUrdf(
        "robot_model/ground/sphere.urdf", "robot_model/ground",
        pin.JointModelFreeFlyer())
    icp_viz = MeshcatVisualizer(icp_model, icp_collision_model,
                                icp_visual_model)
    icp_viz.initViewer(viz.viewer)
    icp_viz.loadViewerModel(rootNodeName="icp", color=[0., 0., 1., 0.5])
    icp_viz_q = pin.neutral(icp_model)

    icp_des_model, icp_des_collision_model, icp_des_visual_model = pin.buildModelsFromUrdf(
        "robot_model/ground/sphere.urdf", "robot_model/ground",
        pin.JointModelFreeFlyer())
    icp_des_viz = MeshcatVisualizer(icp_des_model, icp_des_collision_model,
                                    icp_des_visual_model)
    icp_des_viz.initViewer(viz.viewer)
    icp_des_viz.loadViewerModel(rootNodeName="icp_des",
                                color=[1., 0., 0., 0.5])
    icp_des_viz_q = pin.neutral(icp_des_model)

    cmp_model, cmp_col, cmp_vis = pin.buildModelsFromUrdf(
        "robot_model/ground/sphere.urdf", "robot_model/ground",
        pin.JointModelFreeFlyer())
    cmp_viz = MeshcatVisualizer(cmp_model, cmp_col, cmp_vis)
    cmp_viz.initViewer(viz.viewer)
    cmp_viz.loadViewerModel(rootNodeName="cmp_des", color=[0., 1., 0., 0.5])
    cmp_des_q = pin.neutral(cmp_model)

msg = pnc_msg()

while True:

    encoded_msg = pnc_socket.recv()
    msg.ParseFromString(encoded_msg)

    # receive ground truth data (if in simulation)
    if not config["b_exp"]:
        encoded_gt_msg = gt_socket.recv()
        gt_msg.ParseFromString(encoded_gt_msg)

    # save pkl
    data_saver.add('time', msg.time)
    data_saver.add('phase', msg.phase)

    data_saver.add('l_knee_int_frc_cmd', msg.l_knee_int_frc_cmd)
    data_saver.add('r_knee_int_frc_cmd', msg.r_knee_int_frc_cmd)

    data_saver.add('task_com_pos_des', list(msg.task_com_pos_des))
    data_saver.add('task_com_vel_des', list(msg.task_com_vel_des))
    data_saver.add('task_com_acc_des', list(msg.task_com_acc_des))
    data_saver.add('task_com_pos', list(msg.task_com_pos))
    data_saver.add('task_com_vel', list(msg.task_com_vel))

    data_saver.add('task_com_pos_des_local', list(msg.task_com_pos_des_local))
    data_saver.add('task_com_vel_des_local', list(msg.task_com_vel_des_local))
    data_saver.add('task_com_acc_des_local', list(msg.task_com_acc_des_local))
    data_saver.add('task_com_pos_local', list(msg.task_com_pos_local))
    data_saver.add('task_com_vel_local', list(msg.task_com_vel_local))

    data_saver.add('task_torso_ori_pos_des', list(msg.task_torso_ori_pos_des))
    data_saver.add('task_torso_ori_vel_des', list(msg.task_torso_ori_vel_des))
    data_saver.add('task_torso_ori_acc_des', list(msg.task_torso_ori_acc_des))
    data_saver.add('task_torso_ori_pos', list(msg.task_torso_ori_pos))
    data_saver.add('task_torso_ori_vel', list(msg.task_torso_ori_vel))

    data_saver.add('task_torso_rpy_pos_des',
                   list(util.quat_to_rpy(msg.task_torso_ori_pos_des)))
    data_saver.add('task_torso_rpy_pos',
                   list(util.quat_to_rpy(msg.task_torso_ori_pos)))

    data_saver.add('task_torso_ori_pos_des_local',
                   list(msg.task_torso_ori_pos_des_local))
    data_saver.add('task_torso_ori_vel_des_local',
                   list(msg.task_torso_ori_vel_des_local))
    data_saver.add('task_torso_ori_acc_des_local',
                   list(msg.task_torso_ori_acc_des_local))
    data_saver.add('task_torso_ori_pos_local',
                   list(msg.task_torso_ori_pos_local))
    data_saver.add('task_torso_ori_vel_local',
                   list(msg.task_torso_ori_vel_local))

    data_saver.add('task_torso_rpy_pos_des_local',
                   list(util.quat_to_rpy(msg.task_torso_ori_pos_des_local)))
    data_saver.add('task_torso_rpy_pos_local',
                   list(util.quat_to_rpy(msg.task_torso_ori_pos_local)))

    data_saver.add('task_rfoot_lin_pos_des', list(msg.task_rfoot_lin_pos_des))
    data_saver.add('task_rfoot_lin_vel_des', list(msg.task_rfoot_lin_vel_des))
    data_saver.add('task_rfoot_lin_acc_des', list(msg.task_rfoot_lin_acc_des))
    data_saver.add('task_rfoot_lin_pos', list(msg.task_rfoot_lin_pos))
    data_saver.add('task_rfoot_lin_vel', list(msg.task_rfoot_lin_vel))

    data_saver.add('task_rfoot_lin_pos_des_local',
                   list(msg.task_rfoot_lin_pos_des_local))
    data_saver.add('task_rfoot_lin_vel_des_local',
                   list(msg.task_rfoot_lin_vel_des_local))
    data_saver.add('task_rfoot_lin_acc_des_local',
                   list(msg.task_rfoot_lin_acc_des_local))
    data_saver.add('task_rfoot_lin_pos_local',
                   list(msg.task_rfoot_lin_pos_local))
    data_saver.add('task_rfoot_lin_vel_local',
                   list(msg.task_rfoot_lin_vel_local))

    data_saver.add('task_rfoot_ori_pos_des', list(msg.task_rfoot_ori_pos_des))
    data_saver.add('task_rfoot_ori_vel_des', list(msg.task_rfoot_ori_vel_des))
    data_saver.add('task_rfoot_ori_acc_des', list(msg.task_rfoot_ori_acc_des))
    data_saver.add('task_rfoot_ori_pos', list(msg.task_rfoot_ori_pos))
    data_saver.add('task_rfoot_ori_vel', list(msg.task_rfoot_ori_vel))

    data_saver.add('task_rfoot_rpy_pos_des',
                   list(util.quat_to_rpy(msg.task_rfoot_ori_pos_des)))
    data_saver.add('task_rfoot_rpy_pos',
                   list(util.quat_to_rpy(msg.task_rfoot_ori_pos)))

    data_saver.add('task_rfoot_ori_pos_des_local',
                   list(msg.task_rfoot_ori_pos_des_local))
    data_saver.add('task_rfoot_ori_vel_des_local',
                   list(msg.task_rfoot_ori_vel_des_local))
    data_saver.add('task_rfoot_ori_acc_des_local',
                   list(msg.task_rfoot_ori_acc_des_local))
    data_saver.add('task_rfoot_ori_pos_local',
                   list(msg.task_rfoot_ori_pos_local))
    data_saver.add('task_rfoot_ori_vel_local',
                   list(msg.task_rfoot_ori_vel_local))

    data_saver.add('task_rfoot_rpy_pos_des_local',
                   list(util.quat_to_rpy(msg.task_rfoot_ori_pos_des_local)))
    data_saver.add('task_rfoot_rpy_pos_local',
                   list(util.quat_to_rpy(msg.task_rfoot_ori_pos_local)))

    data_saver.add('task_lfoot_lin_pos_des', list(msg.task_lfoot_lin_pos_des))
    data_saver.add('task_lfoot_lin_vel_des', list(msg.task_lfoot_lin_vel_des))
    data_saver.add('task_lfoot_lin_acc_des', list(msg.task_lfoot_lin_acc_des))
    data_saver.add('task_lfoot_lin_pos', list(msg.task_lfoot_lin_pos))
    data_saver.add('task_lfoot_lin_vel', list(msg.task_lfoot_lin_vel))

    data_saver.add('task_lfoot_lin_pos_des_local',
                   list(msg.task_lfoot_lin_pos_des_local))
    data_saver.add('task_lfoot_lin_vel_des_local',
                   list(msg.task_lfoot_lin_vel_des_local))
    data_saver.add('task_lfoot_lin_acc_des_local',
                   list(msg.task_lfoot_lin_acc_des_local))
    data_saver.add('task_lfoot_lin_pos_local',
                   list(msg.task_lfoot_lin_pos_local))
    data_saver.add('task_lfoot_lin_vel_local',
                   list(msg.task_lfoot_lin_vel_local))

    data_saver.add('task_lfoot_ori_pos_des', list(msg.task_lfoot_ori_pos_des))
    data_saver.add('task_lfoot_ori_vel_des', list(msg.task_lfoot_ori_vel_des))
    data_saver.add('task_lfoot_ori_acc_des', list(msg.task_lfoot_ori_acc_des))
    data_saver.add('task_lfoot_ori_pos', list(msg.task_lfoot_ori_pos))
    data_saver.add('task_lfoot_ori_vel', list(msg.task_lfoot_ori_vel))

    data_saver.add('task_lfoot_rpy_pos_des',
                   list(util.quat_to_rpy(msg.task_lfoot_ori_pos_des)))
    data_saver.add('task_lfoot_rpy_pos',
                   list(util.quat_to_rpy(msg.task_lfoot_ori_pos)))

    data_saver.add('task_lfoot_ori_pos_des_local',
                   list(msg.task_lfoot_ori_pos_des_local))
    data_saver.add('task_lfoot_ori_vel_des_local',
                   list(msg.task_lfoot_ori_vel_des_local))
    data_saver.add('task_lfoot_ori_acc_des_local',
                   list(msg.task_lfoot_ori_acc_des_local))
    data_saver.add('task_lfoot_ori_pos_local',
                   list(msg.task_lfoot_ori_pos_local))
    data_saver.add('task_lfoot_ori_vel_local',
                   list(msg.task_lfoot_ori_vel_local))

    data_saver.add('task_lfoot_rpy_pos_des_local',
                   list(util.quat_to_rpy(msg.task_lfoot_ori_pos_des_local)))
    data_saver.add('task_lfoot_rpy_pos_local',
                   list(util.quat_to_rpy(msg.task_lfoot_ori_pos_local)))

    data_saver.add('task_upper_body_pos_des',
                   list(msg.task_upper_body_pos_des))
    data_saver.add('task_upper_body_vel_des',
                   list(msg.task_upper_body_vel_des))
    data_saver.add('task_upper_body_acc_des',
                   list(msg.task_upper_body_acc_des))
    data_saver.add('task_upper_body_pos', list(msg.task_upper_body_pos))
    data_saver.add('task_upper_body_vel', list(msg.task_upper_body_vel))

    data_saver.add('task_cam_vel_des', list(msg.task_cam_vel_des))
    data_saver.add('task_cam_vel', list(msg.task_cam_vel))
    data_saver.add('task_cam_acc_des', list(msg.task_cam_vel_des))

    data_saver.add('cmd_lfoot_rf', list(msg.cmd_lfoot_rf))
    data_saver.add('cmd_rfoot_rf', list(msg.cmd_rfoot_rf))
    data_saver.add('cmd_joint_positions', list(msg.cmd_joint_positions))
    data_saver.add('cmd_joint_velocities', list(msg.cmd_joint_velocities))
    data_saver.add('cmd_joint_torques', list(msg.cmd_joint_torques))

    data_saver.add('joint_positions', list(msg.joint_positions))
    data_saver.add('joint_velocities', list(msg.joint_velocities))

    data_saver.add('com_vel_est', list(msg.com_vel_est))
    data_saver.add('com_vel_raw', list(msg.com_vel_raw))
    data_saver.add('imu_ang_vel_est', list(msg.imu_ang_vel_est))
    data_saver.add('imu_ang_vel_raw', list(msg.imu_ang_vel_raw))
    data_saver.add('cam_est', list(msg.cam_est))
    data_saver.add('cam_raw', list(msg.cam_raw))

    data_saver.add('icp_des', list(msg.icp_des))
    data_saver.add('icp', list(msg.icp))
    data_saver.add('icp_dot_des', list(msg.icp_dot_des))
    data_saver.add('icp_dot', list(msg.icp_dot))

    # values from KF state estimator
    data_saver.add('base_quat_kf', list(msg.base_quat_kf))
    data_saver.add('base_euler_kf', list(msg.base_euler_kf))
    data_saver.add('base_pos_kf', list(msg.base_pos_kf))
    data_saver.add('base_vel_kf', list(msg.base_vel_kf))
    data_saver.add('imu_accel', list(msg.imu_accel))

    # kinematics-based state estimator
    # data_saver.add('base_joint_pos_est', list(msg.base_joint_pos_est))
    # data_saver.add('base_joint_quat_est', list(msg.base_joint_quat_est))
    # data_saver.add('base_joint_euler_est', list(msg.base_joint_euler_est))

    data_saver.add('base_com_pos', list(msg.base_com_pos))
    data_saver.add('base_com_quat', list(msg.base_com_quat))

    # update feet contacts
    data_saver.add('lfoot_contact', list(msg.lfoot_contact))
    data_saver.add('rfoot_contact', list(msg.rfoot_contact))
    data_saver.add('lf_contact', list(msg.lf_contact))
    data_saver.add('rf_contact', list(msg.rf_contact))

    data_saver.add('kf_time_ms', msg.kf_time_ms)

    # ground truth from pybullet
    if not config["b_exp"]:
        data_saver.add('base_joint_pos', list(gt_msg.base_joint_pos))
        data_saver.add('base_com_pos_py', list(gt_msg.base_com_pos_py))
        data_saver.add('base_joint_quat',
                       list(gt_msg.base_joint_quat))  # (w, x, y, z) order
        data_saver.add('base_joint_lin_vel', list(gt_msg.base_joint_lin_vel))
        data_saver.add('base_joint_ang_vel', list(gt_msg.base_joint_ang_vel))
        data_saver.add('lf_normal_force', list(gt_msg.lf_normal_force))
        data_saver.add('rf_normal_force', list(gt_msg.rf_normal_force))
        data_saver.add('b_lf_force_contact', list(gt_msg.b_lf_force_contact))
        data_saver.add('b_rf_force_contact', list(gt_msg.b_rf_force_contact))
        data_saver.add('des_cmp', list(msg.des_cmp))

    data_saver.add('base_joint_lin_vel', list(msg.base_joint_lin_vel))

    data_saver.advance()

    # publish back for plot juggler
    pj_socket.send_string(json.dumps(data_saver.history))

    # publish joint positions for meshcat
    if args.b_visualize:
        if not config["b_exp"]:
            vis_q[0:3] = np.array(gt_msg.base_joint_pos)  # << base pos
            vis_q[3] = gt_msg.base_joint_quat[0]  # << quaternion x
            vis_q[4] = gt_msg.base_joint_quat[1]  # << quaternion y
            vis_q[5] = gt_msg.base_joint_quat[2]  # << quaternion z
            vis_q[6] = gt_msg.base_joint_quat[3]  # << quaternion w
            vis_q[7:] = np.array(msg.joint_positions)  # << joint pos
            viz.display(vis_q)

        vis_q_kf[0:3] = np.array(msg.base_pos_kf)
        vis_q_kf[3] = np.array(msg.base_quat_kf[1])
        vis_q_kf[4] = np.array(msg.base_quat_kf[2])
        vis_q_kf[5] = np.array(msg.base_quat_kf[3])
        vis_q_kf[6] = np.array(msg.base_quat_kf[0])
        vis_q_kf[7:] = np.array(msg.joint_positions)  # << joint pos

        icp_viz_q[0] = msg.icp[0]
        icp_viz_q[1] = msg.icp[1]
        icp_viz_q[2] = 0.

        icp_des_viz_q[0] = msg.icp_des[0]
        icp_des_viz_q[1] = msg.icp_des[1]
        icp_des_viz_q[2] = 0.

        cmp_des_q[0] = msg.des_cmp[0]
        cmp_des_q[1] = msg.des_cmp[1]
        cmp_des_q[2] = 0.

        viz_kf.display(vis_q_kf)
        icp_viz.display(icp_viz_q)
        icp_des_viz.display(icp_des_viz_q)
        cmp_viz.display(cmp_des_q)
