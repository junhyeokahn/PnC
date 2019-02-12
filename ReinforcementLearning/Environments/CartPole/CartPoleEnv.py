import sys
import os
import yaml

import gym
from gym import error, spaces, utils
from gym.utils import seeding

import numpy as np


class CartPoleEnv(gym.Env):
    """
    Dummy gym environment for action_space and observation_space
    """
    metadata = {'render.modes': ['human']}

    def __init__(self):
        cfg_path = os.getcwd() + '/Config/CartPole/TEST/RL_TEST.yaml'
        with open(cfg_path) as f:
            config = yaml.safe_load(f)
            action_lower_bound = config['control_configuration']['nn_ctrl']['action_lower_bound']
            action_upper_bound = config['control_configuration']['nn_ctrl']['action_upper_bound']
            obs_lower_bound = np.array(config['control_configuration']['nn_ctrl']['obs_lower_bound'])
            obs_upper_bound = np.array(config['control_configuration']['nn_ctrl']['obs_upper_bound'])
        self.action_space = spaces.Box(low=action_lower_bound, high=action_upper_bound, shape=(1,), dtype=np.float32)
        self.observation_space = spaces.Box(low=obs_lower_bound, high=obs_upper_bound, dtype=np.float32)
    def step(self, action):
        pass

    def reset(self):
        pass

    def render(self, mode='human', close=False):
        pass

