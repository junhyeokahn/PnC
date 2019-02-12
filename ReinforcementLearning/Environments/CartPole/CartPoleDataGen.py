import zmq
import sys
import os
import time
import numpy as np

from ProcessManager import ProcessManager
from cart_pole_msg_pb2 import *

class CartPoleDataGen(object):
    def __init__(self, ip, username, password, horizon, verbose=0):
        self.process_manager = ProcessManager(ip, username, password, \
                'cd ~/Repository/PnC/build/bin && ./run_cart_pole', \
                'pkill cart_pole', verbose = verbose)
        self.horizon = horizon
        self.verbose = verbose

        fileHandler = open ("PnC/CartPolePnC/CartPoleDefinition.hpp", "r")
        listOfLines = fileHandler.readlines()
        for line in listOfLines:
            if (len(line.split(' ')) > 5) and line.split(' ')[6] == 'IpSubPub':
                IP_RL_SUB_PUB= line.split(' ')[-1].split('\n')[0].split('"')[1]
            if (len(line.split(' ')) > 5) and line.split(' ')[6] == 'IpReqRep':
                IP_RL_REQ_REP= line.split(' ')[-1].split('\n')[0].split('"')[1]
        fileHandler.close()

        # Constructe zmq socket and connect
        self.context = zmq.Context()
        self.data_socket = self.context.socket(zmq.SUB)
        self.data_socket.connect(IP_RL_SUB_PUB)
        self.data_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.policy_valfn_socket = self.context.socket(zmq.REQ)
        self.policy_valfn_socket.connect(IP_RL_REQ_REP)

    def run_experiment(self, policy_param, valfn_param):
        self.process_manager.quit_process()
        self.process_manager.execute_process()
        self.pair_and_sync()
        # ======================================================================
        # send policy
        # ======================================================================
        pb_policy_param = NeuralNetworkParam()
        for l_idx in range(int((len(policy_param)-1)/2)):
            weight = policy_param[2*l_idx]
            bias = policy_param[2*l_idx+1]
            layer = pb_policy_param.layers.add()
            layer.num_input = weight.shape[0]
            layer.num_output = weight.shape[1]
            for w_row in range(weight.shape[0]):
                for w_col in range(weight.shape[1]):
                    layer.weight.append(weight[w_row, w_col])
            for b_idx in range(bias.shape[0]):
                layer.bias.append(bias[b_idx])
            layer.act_fn = NeuralNetworkParam.Tanh
        pb_policy_param.stochastic = True
        for action_idx in range(policy_param[-1].shape[0]):
            pb_policy_param.logstd.append((policy_param[-1])[action_idx])
        pb_policy_param_serialized = pb_policy_param.SerializeToString()
        self.policy_valfn_socket.send(pb_policy_param_serialized)
        self.policy_valfn_socket.recv()
        if self.verbose >= 1:
            print("[[Policy is set]]")

        # ======================================================================
        # send value functions
        # ======================================================================
        pb_valfn_param = NeuralNetworkParam()
        for l_idx in range(int((len(valfn_param))/2)):
            weight = valfn_param[2*l_idx]
            bias = valfn_param[2*l_idx+1]
            layer = pb_valfn_param.layers.add()
            layer.num_input = weight.shape[0]
            layer.num_output = weight.shape[1]
            for w_row in range(weight.shape[0]):
                for w_col in range(weight.shape[1]):
                    layer.weight.append(weight[w_row, w_col])
            for b_idx in range(bias.shape[0]):
                layer.bias.append(bias[b_idx])
            if l_idx == int((len(valfn_param))/2)-1:
                layer.act_fn = NeuralNetworkParam.NONE
            else:
                layer.act_fn = NeuralNetworkParam.Tanh
        pb_policy_param.stochastic = False
        pb_valfn_param_serialized = pb_valfn_param.SerializeToString()
        self.policy_valfn_socket.send(pb_valfn_param_serialized)
        self.policy_valfn_socket.recv()
        if self.verbose >= 1:
            print("[[Value function is set]]")

    def get_data_segment(self, sess, tf_policy_var, tf_valfn_var):
        policy_param = sess.run(tf_policy_var)
        valfn_param = sess.run(tf_valfn_var)
        self.run_experiment(policy_param, valfn_param);
        count_list = []
        ob_list = []
        rew_list = []
        true_rew_list = []
        vpred_list = []
        action_list = []
        prev_action_list = []
        prev_action = 0
        done_list = []

        cur_ep_ret = 0  # return in current episode
        current_it_len = 0  # len of current iteration
        cur_ep_true_ret = 0
        ep_true_ret_list = []
        ep_ret_list = []  # returns of completed episodes in this segment
        ep_len_list = []  # Episode lengths

        b_first = True

        while(True):

            if (len(ob_list) < self.horizon):
                pb_data_set = DataSet()
                zmq_msg = self.data_socket.recv()
                pb_data_set.ParseFromString(zmq_msg)
                rew_list.append(pb_data_set.reward)
                true_rew_list.append(pb_data_set.reward)
                ob_list.append(pb_data_set.observation)
                vpred_list.append(pb_data_set.vpred)
                done_list.append(pb_data_set.done)
                action_list.append(pb_data_set.action)
                prev_action_list.append(prev_action)
                prev_action = pb_data_set.action

                cur_ep_ret += pb_data_set.reward
                count_list.append(pb_data_set.count)
                current_it_len = pb_data_set.count
                cur_ep_true_ret += pb_data_set.reward
                if b_first:
                    if pb_data_set.count != 0:
                        print("[[Error]] Count does not start from zero!!")
                    b_first = False

                if pb_data_set.done:
                    ep_ret_list.append(cur_ep_ret)
                    ep_true_ret_list.append(cur_ep_true_ret)
                    ep_len_list.append(current_it_len)
                    cur_ep_ret = 0
                    cur_ep_true_ret = 0
                    current_it_len = 0

                    self.process_manager.quit_process()
                    self.run_experiment(policy_param, valfn_param)

            else:
                self.process_manager.quit_process()
                break;

        ob_list = np.array(ob_list)
        rew_list = np.array(rew_list)
        true_rew_list = np.array(true_rew_list)
        vpred_list = np.array(vpred_list)
        action_list = np.array(action_list).reshape([self.horizon, 1])
        prev_action_list = np.array(prev_action_list).reshape([self.horizon, 1])
        nextvpred = vpred_list[-1] * ( 1 - done_list[-1] )

        if ep_ret_list == 0:
            current_it_timesteps = current_it_len
        else:
            current_it_timesteps = sum(ep_len_list) + current_it_len

        if self.verbose >= 2:
            print("=======================data generation========================")
            print("current_it_timesteps")
            print(current_it_timesteps)
            print("current_it_len")
            print(current_it_len)
            print("count list")
            print(count_list)
            print("=======================data generation========================")

        return {'ob': ob_list, 'rew': rew_list, 'dones':done_list, 'true_rew': true_rew_list,
                'vpred': vpred_list, 'ac': action_list, 'prevac':prev_action_list,
                'nextvpred': nextvpred, 'ep_rets':ep_ret_list,'ep_lens':ep_len_list,
                'ep_true_rets':ep_true_ret_list, 'total_timestep':current_it_timesteps}

    def pair_and_sync(self):
        while True:
            try:
                zmq_msg = self.data_socket.recv(zmq.DONTWAIT)
            except zmq.ZMQError as e:
                if e.errno == zmq.EAGAIN:
                    self.policy_valfn_socket.send(b"nope")
                    self.policy_valfn_socket.recv()
                else:
                    raise
            else:
                self.policy_valfn_socket.send(b"world")
                self.policy_valfn_socket.recv()
                break;
        if self.verbose >= 1 :
            print("[[Sockets are all paired and synced]]")
