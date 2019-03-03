from MyGym.WalkerBaseEnv import WalkerBaseBulletEnv
from MyGym.Robots import Humanoid

class HumanoidBulletEnv(WalkerBaseBulletEnv):
    def __init__(self, robot=Humanoid(), render=False):
        self.robot = robot
        WalkerBaseBulletEnv.__init__(self, self.robot, render)
        self.electricity_cost = 4.25 * WalkerBaseBulletEnv.electricity_cost
        self.stall_torque_cost = 4.25 * WalkerBaseBulletEnv.stall_torque_cost

