import yaml
import sys
import os
sys.path.append(os.getcwd() + '/PnC/ReinforcementLearning')
sys.path.append(os.getcwd() + '/PnC/ReinforcementLearning/GymEnv')
sys.path.append(os.getcwd() + '/PnC/CartPolePnC')
sys.path.append(os.getcwd() + '/build/Messages')

from ProximalPolicyOptimization import PPO
from CartPoleEnv import CartPoleEnv
from CartPoleDataGen import CartPoleDataGen

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--password", type=str)
    args = parser.parse_args()

    env = CartPoleEnv()
    cart_pole_data_gen = CartPoleDataGen('localhost', 'junhyeokahn', args.password, 256)
    model = PPO('MlpPolicy', env, cart_pole_data_gen, schedule='linear', verbose=0)
    model.learn(total_timesteps=10000)
