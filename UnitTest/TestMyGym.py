import sys
import os
sys.path.append(os.getcwd()+'/ReinforcementLearning')
import MyGym
import gym
import time

def random_action_test(args):
    count = 0
    if args.render:
        env = gym.make(args.env, render=True)
    else:
        env = gym.make(args.env)
    env.reset()
    sample = env.action_space.sample()
    action = sample
    count = 0
    for i in range(args.steps):
        action = env.action_space.sample()
        action = [0]*len(action)
        obs, rewards, done, _ = env.step(action)
        if done:
            obs = env.reset()
        time.sleep(0.016)

def main():

    import argparse
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--env', help='environment ID')
    parser.add_argument('--render', help='OpenGL Visualizer', type=bool, default=True)
    parser.add_argument('--steps', help='Number of steps', type=int, default=5000)

    args = parser.parse_args()

    random_action_test(args)

if __name__ == '__main__':
    main()
