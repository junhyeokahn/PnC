from MyGym.WalkerBaseEnv import WalkerBaseBulletEnv
from MyGym.Robots import Hopper


class HopperBulletEnv(WalkerBaseBulletEnv):
    def __init__(self, render=False):
        self.robot = Hopper()
        WalkerBaseBulletEnv.__init__(self, self.robot, render)

