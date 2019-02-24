import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)
import pybullet_envs
import gym
import argparse
import pybullet as p


def test(args):
	count = 0
	# env = gym.make(args.env)
	env = gym.make('MyBulletCartPole-v0',**{'renders':True})
	print("args.render=",args.render)
	env.reset()
	print("action space:")
	sample = env.action_space.sample()
	action = sample
	print("action=")
	print(action)
	for i in range(args.steps):
		obs,rewards,done,_ =env.step(action)
		print("obs=")
		print(obs)
		print("rewards")
		print (rewards)
		print ("done")
		print(done)


def main():
    import argparse
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    # parser.add_argument('--env', help='environment ID', default='AntBulletEnv-v0')
    parser.add_argument('--env', help='environment ID', default='CartPoleBulletEnv-v1')
    parser.add_argument('--seed', help='RNG seed', type=int, default=0)
    parser.add_argument('--render', help='OpenGL Visualizer', type=int, default=0)
    parser.add_argument('--rgb',help='rgb_array gym rendering',type=int, default=0)
    parser.add_argument('--resetbenchmark',help='Repeat reset to show reset performance',type=int, default=0)
    parser.add_argument('--steps', help='Number of steps', type=int, default=1)
    
    args = parser.parse_args()
    test(args)

if __name__ == '__main__':
    main()
