import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)
import pybullet_envs
import gym
import argparse
import pybullet as p
import pybullet_envs
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
        # action = [0]*len(action)
        obs, rewards, done, _ = env.step(action)
        if done:
            obs = env.reset()
        # time.sleep(1./240.)
        time.sleep(0.016)

def computation_time_test(args):
    t_0 = time.time()
    count = 0
    env = gym.make(args.env)
    env.reset()
    sample = env.action_space.sample()
    t_1 = time.time()
    print("initialization time : ", t_1 - t_0)
    for i in range(args.steps):
        action = env.action_space.sample()
        action = [0]*len(action)
        obs, rewards, done, _ = env.step(action)
        # if done:
            # obs = env.reset()
    t_2 = time.time()
    print("computation time : ", t_2 - t_1)

def main():
    import argparse
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--env', help='environment ID')
    parser.add_argument('--render', help='OpenGL Visualizer', type=bool, default=True)
    parser.add_argument('--steps', help='Number of steps', type=int, default=5000)

    args = parser.parse_args()

    ### TEST
    random_action_test(args)
    # computation_time_test(args)

if __name__ == '__main__':
    main()
