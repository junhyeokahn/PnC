import sys
import os
sys.path.append(os.getcwd() + '/../PnC/ReinforcementLearning')

import PPO

os.environ['KMP_DUPLICATE_LIB_OK']='True'

if __name__ == '__main__':
    model = PPO.PPO('MlpPolicy', 'CartPole-v1', schedule='linear', verbose=0)
    model.learn(total_timesteps=10000)

'''
from stable_baselines import PPO1

import os
os.environ['KMP_DUPLICATE_LIB_OK']='True'

if __name__ == '__main__':
    model = PPO1('MlpPolicy', 'CartPole-v1', schedule='linear', verbose=0)
    model.learn(total_timesteps=10000)
'''
