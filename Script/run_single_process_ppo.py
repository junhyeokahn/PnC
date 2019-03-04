import sys
import multiprocessing
import os
import os.path as osp
sys.path.append(os.getcwd()+'/ReinforcementLearning')
import MyGym
import gym
from collections import defaultdict
import tensorflow as tf
import numpy as np


from baselines.common.vec_env.vec_video_recorder import VecVideoRecorder
from baselines.common.vec_env.vec_frame_stack import VecFrameStack
from baselines.common.cmd_util import make_vec_env, make_env
from baselines.common.tf_util import get_session
from baselines.common.misc_util import get_latest_run_id
from baselines import logger
from importlib import import_module

from baselines.common.vec_env.vec_normalize import VecNormalize

from baselines.ppo2.ppo2 import learn
from baselines.ppo2.model import Model
from baselines.common.policies import build_policy
import time

try:
    from mpi4py import MPI
except ImportError:
    MPI = None

try:
    import pybullet_envs
except ImportError:
    pybullet_envs = None

def do_learn(args):
    # ==========================================================================
    # configure logger
    # ==========================================================================
    if MPI is None or MPI.COMM_WORLD.Get_rank() == 0:
        latest_run_id = get_latest_run_id(args.save_path, 'ppo')
        save_path = osp.join(args.save_path, "{}_{}".format('ppo', latest_run_id+1))
        if not osp.exists(save_path):
            os.makedirs(save_path)
        logger.configure(dir=save_path, format_strs=['stdout', 'csv'])
    else:
        logger.configure(format_strs=[])

    # ==========================================================================
    # Env
    # ==========================================================================
    config = tf.ConfigProto(allow_soft_placement=True,
                                   intra_op_parallelism_threads=1,
                                   inter_op_parallelism_threads=1)
    config.gpu_options.allow_growth = True
    get_session(config=config)
    flatten_dict_observations = True
    env = make_vec_env(args.env, 'custom', args.num_envs, args.seed,
                       reward_scale=args.reward_scale,
                       flatten_dict_observations=flatten_dict_observations)
    # ==========================================================================
    # Network
    # ==========================================================================
    if args.activation == 0:
        activation = None
    elif args.activation == 1:
        activation = tf.tanh
    elif args.activation == 2:
        activation = tf.nn.relu
    else:
        print("Wrong activation function")

    network_kwargs = {'num_layers': args.num_layers,
                      'num_hidden': args.num_hidden,
                      'activation': activation}

    # ==========================================================================
    # Learn
    # ==========================================================================
    nminibatches = args.num_envs * args.num_steps // args.num_data_per_batch
    model = learn(network=args.network, env=env,
                  total_timesteps=args.num_timesteps, seed=args.seed,
                  nsteps=args.num_steps, ent_coef=args.ent_coef, lr=args.lr,
                  vf_coef=args.vf_coef, max_grad_norm=args.max_grad_norm,
                  gamma=args.gamma, lam=args.lam,
                  log_interval=args.log_interval,
                  nminibatches=nminibatches,
                  noptepochs=args.num_epochs, cliprange=args.cliprange,
                  save_interval=args.save_interval,
                  load_path=args.load_path, **network_kwargs)
    env.close()

def do_play(args):
    assert(args.load_path is not None)

    # ==========================================================================
    # Env
    # ==========================================================================
    env = gym.make(args.env, render=True)

    # ==========================================================================
    # Model
    # ==========================================================================
    if args.activation == 0:
        activation = None
    elif args.activation == 1:
        activation = tf.tanh
    elif args.activation == 2:
        activation = tf.nn.relu
    else:
        print("Wrong activation function")
    network_kwargs = {'num_layers': args.num_layers,
                      'num_hidden': args.num_hidden,
                      'activation': activation}

    policy = build_policy(env, args.network, **network_kwargs)
    ob_space = env.observation_space
    ac_space = env.action_space

    nminibatches = args.num_envs * args.num_steps // args.num_data_per_batch

    model = Model(policy=policy, ob_space=ob_space, ac_space=ac_space,
                  nbatch_act=args.num_envs, nbatch_train=args.num_data_per_batch,
                  nsteps=args.num_steps, ent_coef=args.ent_coef,
                  vf_coef=args.vf_coef, max_grad_norm=args.max_grad_norm)
    model.load(args.load_path)

    # ==========================================================================
    # Play
    # ==========================================================================
    obs = env.reset()
    total_rew = 0
    epi_len = 0
    while True:
        actions, _, _, _ = model.step(obs)
        # print("obs : ", obs)
        # print("ac : ", actions)
        obs, rew, done, _ = env.step(np.squeeze(actions))
        total_rew += rew
        epi_len += 1
        if done:
            print("total reward : ", total_rew)
            print("episode length : ", epi_len)
            total_rew = 0
            epi_len = 0
            obs = env.reset()
        # time.sleep(env.env.timeStep)
        time.sleep(0.016)
    env.close()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--play", type=bool, default=False)

    parser.add_argument("--env", type=str)
    parser.add_argument("--num_envs", type=int, default=1)
    parser.add_argument("--reward_scale", type=float, default=1.0)

    parser.add_argument("--network", type=str, default='mlp')
    parser.add_argument("--num_layers", type=int, default=2)
    parser.add_argument("--num_hidden", type=int, default=64)
    parser.add_argument("--activation", type=int, default=1)
    parser.add_argument("--value_network", type=str, default='copy')

    parser.add_argument("--num_timesteps", type=int, default=5e7)
    parser.add_argument("--num_steps", type=int, default=2048)
    parser.add_argument("--ent_coef", type=float, default=0.0)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--vf_coef", type=float, default=0.5)
    parser.add_argument("--max_grad_norm", type=float, default=0.5)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--lam", type=float, default=0.95)
    parser.add_argument("--log_interval", type=int, default=1)
    parser.add_argument("--num_data_per_batch", type=int, default=64)
    parser.add_argument("--num_epochs", type=int, default=10)
    parser.add_argument("--cliprange", type=float, default=0.2)
    parser.add_argument("--seed", type=int)

    parser.add_argument("--load_path", type=str)
    parser.add_argument("--save_interval", type=int, default=10)
    parser.add_argument("--save_path", type=str)
    args = parser.parse_args()

    if args.play:
        do_play(args)
    else:
        do_learn(args)
