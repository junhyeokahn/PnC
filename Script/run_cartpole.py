import datetime
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
    # parser.add_argument("--model_dir", type=str)
    args = parser.parse_args()

    cfg_path = os.getcwd() + '/Config/CartPole/TEST/RL_TEST.yaml'
    with open(cfg_path) as f:
        config = yaml.safe_load(f)
        num_batch = config['control_configuration']['nn_ctrl']['timesteps_per_actorbatch']

    env = CartPoleEnv()
    cart_pole_data_gen = CartPoleDataGen('localhost', 'junhyeokahn', args.password, num_batch, verbose=0)

    # Path for logging
    save_path = os.getcwd() + '/ReinforcementLearning/Environments/CartPole/Log'
    dir_name = 'PPO'

    # Train the model
    model = PPO('MlpPolicy', env, cart_pole_data_gen, schedule='linear',
            verbose=1, timesteps_per_actorbatch=num_batch,
            tensorboard_log=save_path)
    model.learn(total_timesteps=1e6)

    # Save the model
    model.save(save_path, dir_name, new_dir = False)
