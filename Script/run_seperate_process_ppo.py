import datetime
import sys
import os
sys.path.append(os.getcwd() + '/build/Messages')

import gym
from gym.envs.custom import dummy_cartpole
from gym.envs.custom import dummy_draco
from gym import envs
from baselines.ppo2.seperate_process_ppo2 import learn
from baselines.common.misc_util import get_latest_run_id
from baselines import logger
import tensorflow as tf
try:
    from mpi4py import MPI
except ImportError:
    MPI = None

def main(args):
    current_path = os.getcwd()
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--env", type=str)
    parser.add_argument("--num_envs", type=int, default=1)
    parser.add_argument("--password", type=str)

    parser.add_argument("--num_layers", type=int, default=2)
    parser.add_argument("--num_hidden", type=int, default=64)
    parser.add_argument("--activation", type=int, default=1)

    parser.add_argument("--num_timesteps", type=int, default=1e6)
    parser.add_argument("--num_steps", type=int, default=2048)
    parser.add_argument("--ent_coef", type=float, default=0)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--vf_coef", type=float, default=0.5)
    parser.add_argument("--max_grad_norm", type=float, default=0.5)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--lam", type=float, default=0.95)
    parser.add_argument("--num_data_per_batch", type=int, default=64)
    parser.add_argument("--num_optepochs", type=int, default=4)
    parser.add_argument("--cliprange", type=float, default=0.2)

    parser.add_argument("--save_interval", type=int, default=10)
    parser.add_argument("--log_interval", type=int, default=1)
    parser.add_argument("--save_path", type=str)
    parser.add_argument("--load_path", type=str)
    parser.add_argument("--seed", type=int)
    args = parser.parse_args()

    # ==========================================================================
    # network
    # ==========================================================================
    if args.activation == 0:
        activation = None
    elif args.activation == 1:
        activation = tf.tanh
    elif args.activation == 2:
        activation = tf.nn.relu
    else:
        print("Wrong activation function")

    network_kwargs = {'num_layers':args.num_layers,
                      'num_hidden': args.num_hidden,
                      'activation': activation}
    network = 'mlp'

    # ==========================================================================
    # configure logger
    # ==========================================================================
    latest_run_id = get_latest_run_id(args.save_path, 'ppo')
    save_path = os.path.join(args.save_path, "{}_{}".format('ppo', latest_run_id+1))
    if not os.path.exists(save_path):
        os.makedirs(save_path)

    if MPI is None or MPI.COMM_WORLD.Get_rank() == 0:
        rank = 0
        # logger.configure(dir=save_path)
        logger.configure(dir=save_path, format_strs=['stdout', 'csv'])
    else:
        logger.configure(dir=save_path, format_strs=[])
        rank = MPI.COMM_WORLD.Get_rank()

    # ==========================================================================
    # gym env
    # ==========================================================================
    env = envs.make(args.env)

    # ==========================================================================
    # learn model with ppo
    # ==========================================================================
    nminibatches = args.num_envs * args.num_steps // args.num_data_per_batch
    model = learn(env, args.num_envs, network, args.password,
                total_timesteps=args.num_timesteps, nsteps=args.num_steps,
                ent_coef=args.ent_coef, lr=args.lr, vf_coef=args.vf_coef,
                max_grad_norm=args.max_grad_norm, gamma=args.gamma, lam=args.lam,
                log_interval=args.log_interval, nminibatches=num_minibatches,
                noptepochs=args.num_optepochs, cliprange=args.cliprange,
                save_interval=args.save_interval, save_path=save_path,
                load_path=args.load_path, **network_kwargs)

if __name__ == "__main__":
    main(sys.argv)
