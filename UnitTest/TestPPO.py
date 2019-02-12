import yaml
import sys
import os
sys.path.append(os.getcwd() + '/ReinforcementLearning')
sys.path.append(os.getcwd() + '/ReinforcementLearning/Environments/CartPole')
sys.path.append(os.getcwd() + '/ReinforcementLearning/Algorithms')
sys.path.append(os.getcwd() + '/build/Messages')

from ProximalPolicyOptimization import PPO
from CartPoleEnv import CartPoleEnv
from CartPoleDataGen import CartPoleDataGen

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--password", type=str)
    args = parser.parse_args()

    cfg_path = os.getcwd() + '/Config/CartPole/TEST/RL_TEST.yaml'
    with open(cfg_path) as f:
        config = yaml.safe_load(f)
        num_batch = config['control_configuration']['nn_ctrl']['timesteps_per_actorbatch']

    env = CartPoleEnv()
    cart_pole_data_gen = CartPoleDataGen('localhost', 'junhyeokahn', args.password, num_batch, verbose=2)
    model = PPO('MlpPolicy', env, cart_pole_data_gen, schedule='linear',
            verbose=1, timesteps_per_actorbatch=num_batch)
    model.learn(total_timesteps=10000)
