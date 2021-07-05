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

from plot.data_saver import DataSaver

from messages.draco_pb2 import *

with open("config/draco/pnc.yaml", 'r') as stream:
    config = YAML().load(stream)
    addr = config["ip_addr"]

context = zmq.Context()
pnc_socket = context.socket(zmq.SUB)
pnc_socket.connect(addr)
pnc_socket.setsockopt_string(zmq.SUBSCRIBE, "")

pj_context = zmq.Context()
pj_socket = pj_context.socket(zmq.PUB)
pj_socket.bind("tcp://*:9872")

time.sleep(1)

msg = pnc_msg()

data_saver = DataSaver()

while True:

    encoded_msg = pnc_socket.recv()
    msg.ParseFromString(encoded_msg)

    # save pkl
    data_saver.add('time', msg.time)
    data_saver.add('phase', msg.phase)

    data_saver.add('task_com_pos_des', list(msg.task_com_pos_des))
    data_saver.add('task_com_vel_des', list(msg.task_com_vel_des))
    data_saver.add('task_com_acc_des', list(msg.task_com_acc_des))
    data_saver.add('task_com_pos', list(msg.task_com_pos))
    data_saver.add('task_com_vel', list(msg.task_com_vel))

    data_saver.add('task_torso_ori_des', list(msg.task_torso_ori_des))
    data_saver.add('task_torso_ang_vel_des', list(msg.task_torso_ang_vel_des))
    data_saver.add('task_torso_ang_acc_des', list(msg.task_torso_ang_acc_des))
    data_saver.add('task_torso_ori', list(msg.task_torso_ori))
    data_saver.add('task_torso_ang_vel', list(msg.task_torso_ang_vel))

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

    data_saver.add('cmd_lfoot_rf', list(msg.cmd_lfoot_rf))
    data_saver.add('cmd_rfoot_rf', list(msg.cmd_rfoot_rf))

    data_saver.advance()

    # publish back for plot juggler
    pj_socket.send_string(json.dumps(data_saver.history))
