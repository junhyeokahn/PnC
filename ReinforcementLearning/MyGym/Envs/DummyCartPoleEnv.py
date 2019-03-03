import math
import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np

import sys
import os
import yaml

class DummyCartPoleEnv(gym.Env):
    """
    Dummy gym environment for action_space and observation_space
    """
    metadata = {'render.modes': ['human']}

    def __init__(self):
        cfg_path = os.getcwd() + '/Config/CartPole/TEST/RL_TEST.yaml'
        with open(cfg_path) as f:
            config = yaml.safe_load(f)
            self.action_lower_bound = np.array(config['test_configuration']['action_lower_bound'])
            self.action_upper_bound = np.array(config['test_configuration']['action_upper_bound'])
            self.action_scale = config['test_configuration']['action_scale']
            self.obs_lower_bound = np.array(config['test_configuration']['obs_lower_bound'])
            self.obs_upper_bound = np.array(config['test_configuration']['obs_upper_bound'])

        self.action_space = spaces.Box(low=self.action_lower_bound, high=self.action_upper_bound, dtype=np.float32)
        self.observation_space = spaces.Box(low=self.obs_lower_bound, high=self.obs_upper_bound, dtype=np.float32)
    def step(self, action):
        pass

    def reset(self):
        pass

    def render(self, mode='human', close=False):
        pass
