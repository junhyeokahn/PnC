import os
PROJECT_PATH = os.getcwd()

import numpy as np
import pybullet as p

from MyGym.WalkerBase import WalkerBase
from MyGym.RobotBases import URDFBasedRobot


class Draco(WalkerBase, URDFBasedRobot):
    random_yaw = False
    foot_list = ["lAnkle", "rAnkle"]

    def __init__(self):
        n_ac = 8
        n_obs = 8 + 2*n_ac + 2
        WalkerBase.__init__(self, power=1.5)
        URDFBasedRobot.__init__(self, PROJECT_PATH+"/RobotModel/Robot/Draco/FixedDracoSim_PyBullet.urdf", "Torso", action_dim=n_ac, obs_dim=n_obs)

    def alive_bonus(self, z, pitch):
        x, y, z = self.torso.pose().xyz()
        roll, pitch, yaw = self.torso.pose().rpy()

        knees = np.array([j.current_relative_position() for j in [self.jdict["lKnee"], self.jdict["rKnee"]]], dtype=np.float32).flatten()
        knees_at_limit = np.count_nonzero(np.abs(knees[0::2]) > 0.99)
        return +4 - knees_at_limit if ((z > 0.75) and (yaw < np.deg2rad(45))) else -1

    def robot_specific_reset(self, bullet_client):
        WalkerBase.robot_specific_reset(self, bullet_client)
        self.set_base_configuration(yaw_center=0, yaw_random_spread=np.pi)
        self.torso = self.parts["Torso"]

        self.jdict['lKnee'].reset_current_position(0.52, 0)
        self.jdict['rKnee'].reset_current_position(0.52, 0)
        self.jdict['lHipPitch'].reset_current_position(np.deg2rad(-10), 0)
        self.jdict['rHipPitch'].reset_current_position(np.deg2rad(-10), 0)

    def set_base_configuration(self, yaw_center, yaw_random_spread):
        if not self.random_yaw:
            yaw = yaw_center
        else:
            yaw = yaw_center + self.np_random.uniform(low=-yaw_random_spread, high=yaw_random_spread)

        position = [self.start_pos_x, self.start_pos_y, self.start_pos_z + 1.2]
        orientation = [0, np.deg2rad(10), yaw]  # just face random direction, but stay straight otherwise
        self.robot_body.reset_pose(position, p.getQuaternionFromEuler(orientation))
        self.initial_z = 1.2
