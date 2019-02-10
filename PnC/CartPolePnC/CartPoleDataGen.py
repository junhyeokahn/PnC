import zmq
import sys
import os
import time

from ProcessManager import ProcessManager
from cart_pole_msg_pb2 import *

class CartPoleDataGen(object):
    def __init__(self, ip, username, password, horizon):
        self.process_manager = ProcessManager(ip, username, password, \
                'cd ~/Repository/PnC/build/bin && ./run_cart_pole', \
                'pkill cart_pole')
        self.horizon = horizon

        fileHandler = open ("Configuration.h", "r")
        listOfLines = fileHandler.readlines()
        for line in listOfLines:
            if len(line.split(' ')) > 1:
                if line.split(' ')[1] == 'IP_RL_REQ_REP':
                    IP_RL_REQ_REP = line.split(' ')[2].split('\n')[0].split('"')[1]
                if line.split(' ')[1] == 'IP_RL_SUB_PUB':
                    IP_RL_SUB_PUB = line.split(' ')[2].split('\n')[0].split('"')[1]
        fileHandler.close()

        # Constructe zmq socket and connect
        self.context = zmq.Context()
        self.data_socket = self.context.socket(zmq.SUB)
        self.data_socket.connect(IP_RL_SUB_PUB)
        self.data_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.policy_socket = self.context.socket(zmq.REQ)
        self.policy_socket.connect(IP_RL_REQ_REP)

    def run_experiment(self, policy):
        self.process_manager.execute_process()
        self.pair_and_sync()
        # ======================================================================
        # send policy
        # ======================================================================
        pb_policy_param = StochasticPolicyParam()
        layer = []
        for l_idx in range(int((len(policy)-1)/2)):
            weight = policy[2*l_idx]
            bias = policy[2*l_idx+1]
            layer = pb_policy_param.layers.add()
            layer.num_input = weight.shape[0]
            layer.num_output = weight.shape[1]
            for w_row in range(weight.shape[0]):
                for w_col in range(weight.shape[1]):
                    layer.weight.append(weight[w_row, w_col])
            for b_idx in range(bias.shape[0]):
                layer.bias.append(bias[b_idx])
            layer.act_fn = StochasticPolicyParam.Tanh
        for action_idx in range(policy[-1].shape[0]):
            pb_policy_param.logstd.append((policy[-1])[action_idx])
        pb_policy_param_serialized = pb_policy_param.SerializeToString()
        self.policy_socket.send(pb_policy_param_serialized)
        self.policy_socket.recv()

    def get_data_segment(self, policy):
        self.run_experiment(policy);
        ob_list = []
        rew_list = []
        while(True):
            pb_data_set = DataSet()
            print("receiving data set")
            zmq_msg = self.data_socket.recv()

            pb_data_set.ParseFromString(zmq_msg)
            rew_list.append(pb_data_set.reward)
            ob_list.append(pb_data_set.observation)
            if (len(ob_list) <= self.horizon):
                if pb_data_set.done:
                    self.process_manager.quit_process()
                    time.sleep(0.5)
                    self.run_experiment(policy)
            else:
                break;


        # return {"ob": observation}
        pass

    def pair_and_sync(self):
        while True:
            try:
                zmq_msg = self.data_socket.recv(zmq.DONTWAIT)
            except zmq.ZMQError as e:
                if e.errno == zmq.EAGAIN:
                    self.policy_socket.send(b"nope")
                    self.policy_socket.recv()
                else:
                    raise
            else:
                self.policy_socket.send(b"world")
                self.policy_socket.recv()
                break;
        print("Sockets are all paired and synced")
