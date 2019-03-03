from MyGym.WalkerBaseEnv import WalkerBaseBulletEnv
from MyGym.Robots import Draco
from MyGym.Scenes import StadiumScene

class DracoBulletEnv(WalkerBaseBulletEnv):
    def __init__(self, render=False):
        self.robot = Draco()
        WalkerBaseBulletEnv.__init__(self, self.robot, render)

    def create_single_player_scene(self, bullet_client):
        self.stadium_scene = StadiumScene(bullet_client, gravity=9.8, timestep=0.0165/8, frame_skip=8)   # 8 instead of 4 here
        return self.stadium_scene

    def robot_specific_reset(self):
        self.robot.robot_specific_reset()
