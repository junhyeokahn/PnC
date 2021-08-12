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

from plot.data_saver import DataSaver

from messages.fixed_draco_pb2 import *

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--b_visualize", type=bool, default=False)
args = parser.parse_args()

with open("config/fixed_draco/pnc.yaml", 'r') as stream:
    config = YAML().load(stream)
    addr = config["ip_addr"]

context = zmq.Context()
pnc_socket = context.socket(zmq.SUB)
pnc_socket.connect(addr)
pnc_socket.setsockopt_string(zmq.SUBSCRIBE, "")

# pj_context = zmq.Context()
# pj_socket = pj_context.socket(zmq.PUB)
# pj_socket.bind("tcp://*:9872")

data_saver = DataSaver()

if args.b_visualize:
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        "robot_model/draco/draco.urdf", "robot_model/draco",
        pin.JointModelFreeFlyer())
    viz = MeshcatVisualizer(model, collision_model, visual_model)
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

msg = pnc_msg()

while True:

    encoded_msg = pnc_socket.recv()
    msg.ParseFromString(encoded_msg)

    # save pkl
    data_saver.add('time', msg.time)
    data_saver.add('phase', msg.phase)

    data_saver.add('task_rfoot_pos_des', list(msg.task_rfoot_pos_des))
    data_saver.add('task_rfoot_vel_des', list(msg.task_rfoot_vel_des))
    data_saver.add('task_rfoot_acc_des', list(msg.task_rfoot_acc_des))
    data_saver.add('task_rfoot_pos', list(msg.task_rfoot_pos))
    data_saver.add('task_rfoot_vel', list(msg.task_rfoot_vel))

    data_saver.add('task_rfoot_ori_des', list(msg.task_rfoot_ori_des))
    data_saver.add('task_rfoot_ang_vel_des', list(msg.task_rfoot_ang_vel_des))
    data_saver.add('task_rfoot_ang_acc_des', list(msg.task_rfoot_ang_acc_des))
    data_saver.add('task_rfoot_ori', list(msg.task_rfoot_ori))
    data_saver.add('task_rfoot_ang_vel', list(msg.task_rfoot_ang_vel))

    data_saver.add('task_lfoot_pos_des', list(msg.task_lfoot_pos_des))
    data_saver.add('task_lfoot_vel_des', list(msg.task_lfoot_vel_des))
    data_saver.add('task_lfoot_acc_des', list(msg.task_lfoot_acc_des))
    data_saver.add('task_lfoot_pos', list(msg.task_lfoot_pos))
    data_saver.add('task_lfoot_vel', list(msg.task_lfoot_vel))

    data_saver.add('task_lfoot_ori_des', list(msg.task_lfoot_ori_des))
    data_saver.add('task_lfoot_ang_vel_des', list(msg.task_lfoot_ang_vel_des))
    data_saver.add('task_lfoot_ang_acc_des', list(msg.task_lfoot_ang_acc_des))
    data_saver.add('task_lfoot_ori', list(msg.task_lfoot_ori))
    data_saver.add('task_lfoot_ang_vel', list(msg.task_lfoot_ang_vel))

    data_saver.add('task_rhand_pos_des', list(msg.task_rhand_pos_des))
    data_saver.add('task_rhand_vel_des', list(msg.task_rhand_vel_des))
    data_saver.add('task_rhand_acc_des', list(msg.task_rhand_acc_des))
    data_saver.add('task_rhand_pos', list(msg.task_rhand_pos))
    data_saver.add('task_rhand_vel', list(msg.task_rhand_vel))

    data_saver.add('task_rhand_ori_des', list(msg.task_rhand_ori_des))
    data_saver.add('task_rhand_ang_vel_des', list(msg.task_rhand_ang_vel_des))
    data_saver.add('task_rhand_ang_acc_des', list(msg.task_rhand_ang_acc_des))
    data_saver.add('task_rhand_ori', list(msg.task_rhand_ori))
    data_saver.add('task_rhand_ang_vel', list(msg.task_rhand_ang_vel))

    data_saver.add('task_lhand_pos_des', list(msg.task_lhand_pos_des))
    data_saver.add('task_lhand_vel_des', list(msg.task_lhand_vel_des))
    data_saver.add('task_lhand_acc_des', list(msg.task_lhand_acc_des))
    data_saver.add('task_lhand_pos', list(msg.task_lhand_pos))
    data_saver.add('task_lhand_vel', list(msg.task_lhand_vel))

    data_saver.add('task_lhand_ori_des', list(msg.task_lhand_ori_des))
    data_saver.add('task_lhand_ang_vel_des', list(msg.task_lhand_ang_vel_des))
    data_saver.add('task_lhand_ang_acc_des', list(msg.task_lhand_ang_acc_des))
    data_saver.add('task_lhand_ori', list(msg.task_lhand_ori))
    data_saver.add('task_lhand_ang_vel', list(msg.task_lhand_ang_vel))

    data_saver.add('task_neck_pos_des', list(msg.task_neck_pos_des))
    data_saver.add('task_neck_vel_des', list(msg.task_neck_vel_des))
    data_saver.add('task_neck_acc_des', list(msg.task_neck_acc_des))
    data_saver.add('task_neck_pos', list(msg.task_neck_pos))
    data_saver.add('task_neck_vel', list(msg.task_neck_vel))

    data_saver.add('cmd_joint_positions', list(msg.cmd_joint_positions))
    data_saver.add('cmd_joint_velocities', list(msg.cmd_joint_velocities))
    data_saver.add('cmd_joint_torques', list(msg.cmd_joint_torques))

    data_saver.add('joint_positions', list(msg.joint_positions))
    data_saver.add('joint_velocities', list(msg.joint_velocities))

    data_saver.add('base_joint_quat', list(msg.base_joint_quat))

    data_saver.advance()

    # publish back for plot juggler
    # pj_socket.send_string(json.dumps(data_saver.history))

    # publish joint positions for meshcat
    if args.b_visualize:
        vis_q[0:3] = np.array(msg.base_joint_pos)  # << base pos
        vis_q[3] = msg.base_joint_quat[1]  # << quaternion x
        vis_q[4] = msg.base_joint_quat[2]  # << quaternion y
        vis_q[5] = msg.base_joint_quat[3]  # << quaternion z
        vis_q[6] = msg.base_joint_quat[0]  # << quaternion w
        vis_q[7:] = np.array(msg.joint_positions)  # << joint pos

        viz.display(vis_q)
