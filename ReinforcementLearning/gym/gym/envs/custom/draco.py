import logging
import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import subprocess
import pybullet as p
from pkg_resources import parse_version
import os
PROJECT_PATH = os.getcwd()

logger = logging.getLogger(__name__)

class DracoEnv(gym.Env):
  metadata = {
    'render.modes': ['human', 'rgb_array'],
    'video.frames_per_second' : 50
  }

  def __init__(self, render=False):
    self.debug = False
    self.target_x = 1e3
    self.target_y = 0.0
    # ==========================================================================
    # Renderer
    # ==========================================================================
    self._render = render
    if (render):
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    # ==========================================================================
    # Robot Configuration
    # ==========================================================================
    # Home Pose
    # self.hanging_pos = [0, 0, 1.13]
    # self.hanging_ori = p.getQuaternionFromEuler([0, 0, 0])
    # self.ini_pos = np.array([0, 0, 0, 0, np.pi/2, 0, 0, 0, 0, np.pi/2])
    # Bent Pose
    self.hanging_pos = [0, 0, 1.1]
    self.hanging_ori = p.getQuaternionFromEuler([0, np.deg2rad(15), 0])
    self.ini_pos = np.array([0, 0, np.deg2rad(-30), np.deg2rad(30), np.deg2rad(75),
                             0, 0, np.deg2rad(-30), np.deg2rad(30), np.deg2rad(75)])
    ############################################################################

    if self._render:
        self.draco = p.loadURDF(PROJECT_PATH+"/RobotModel/Robot/Draco/DracoFixed.urdf", self.hanging_pos, self.hanging_ori, useFixedBase=False)
    else:
        self.draco = p.loadURDF(PROJECT_PATH+"/RobotModel/Robot/Draco/DracoFixedNoVisual.urdf", self.hanging_pos, self.hanging_ori, useFixedBase=False)

    p.loadURDF(PROJECT_PATH+"/RobotModel/Ground/plane.urdf")
    self.feet = ['lAnkle', 'rAnkle']

    self.joint_list = {}
    self.link_list = {}
    self.dof_idx = []
    self.dof_lb = []
    self.dof_ub = []
    dof_max_force = []
    active_joint = 0
    for j in range (p.getNumJoints(self.draco)):
        info = p.getJointInfo(self.draco, j)
        joint_name = info[1]
        joint_type = info[2]
        link_name = info[12]
        self.joint_list[joint_name.decode()] = j
        self.link_list[link_name.decode()] = j
        if (joint_type==p.JOINT_PRISMATIC or joint_type==p.JOINT_REVOLUTE):
            self.dof_idx.append(j)
            self.dof_lb.append(info[8])
            self.dof_ub.append(info[9])
            dof_max_force.append(info[10])
            active_joint+=1
    self.n_dof = active_joint

    # ==========================================================================
    # Observationsa \in R^{8+20+2}
    # Actions \in R^{10}
    # ==========================================================================
    obs_high = np.array( [np.inf] * (8+20+2) )
    self.observation_space = spaces.Box(-obs_high, obs_high, dtype=np.float32)
    self.action_space = spaces.Box(np.array([-1]*self.n_dof),
                                   np.array([1]*self.n_dof), dtype=np.float32)
    self.action_sclae = np.array(dof_max_force)

    self.seed()
    self.viewer = None
    self._configure()

  def _configure(self, display=None):
    self.display = display

  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def step(self, action):
    # Apply action
    clamped_action = np.clip(np.array(action), self.action_space.low,
                                               self.action_space.high)
    force = clamped_action * self.action_sclae

    p.setJointMotorControlArray(self.draco, self.dof_idx, p.TORQUE_CONTROL, forces=force)
    p.stepSimulation()

    # Get observation
    obs = self.get_observation()
    if self.debug:
        print("[delta z, sin(angle_to_target), cos(angle_to_target), xdot, ydot, zdot, roll, pitch, jpos, jvel, contact] : \n", obs)

    # Termination condition
    done_info = [False] * 2
    if not np.isfinite(obs).all():
        done_info[0] = True
    if (obs[0] + self.initial_z < 0.95):
        done_info[1] = True
    done = any(done_info)
    if self.debug:
        print("done info : ", done_info)

    # Reward
    if not done:
        alive_bonus = 2.0
    else:
        alive_bonus = 0.0
    action_pen = 1 * (np.square(clamped_action).sum() / self.n_dof)
    joint_limit_pen =  1 * (self.n_joints_at_limit / self.n_dof)
    dist_pen = 2 * (self.walk_target_dist / np.linalg.norm([self.target_x, self.target_y]))

    reward = (alive_bonus - action_pen - joint_limit_pen - dist_pen)

    if self.debug:
        print("alive_bonus : {}, dist pen : {}, action_pen : {}, joint_limit_pen : {}, total_rew : {}".format(alive_bonus, dist_pen, action_pen, joint_limit_pen, reward))
    return obs, reward, done, {}

  # ============================================================================
  # Reset Env
  # ============================================================================
  def reset(self):
    p.resetSimulation()
    p.loadURDF(PROJECT_PATH+"/RobotModel/Ground/plane.urdf")
    if self._render:
        self.draco = p.loadURDF(PROJECT_PATH+"/RobotModel/Robot/Draco/DracoFixed.urdf", self.hanging_pos, self.hanging_ori, useFixedBase=False)
    else:
        self.draco = p.loadURDF(PROJECT_PATH+"/RobotModel/Robot/Draco/DracoFixedNoVisual.urdf", self.hanging_pos, self.hanging_ori, useFixedBase=False)
    self.timeStep = 1./240.
    p.changeDynamics(self.draco, -1, linearDamping=0, angularDamping=0)
    for i in range(self.n_dof):
        p.changeDynamics(self.draco, i, lateralFriction=10)
    p.setJointMotorControlArray(self.draco, self.dof_idx, p.VELOCITY_CONTROL,
                                forces=np.zeros(self.n_dof))
    p.setGravity(0,0, -9.81)
    p.setTimeStep(self.timeStep)
    p.setRealTimeSimulation(0)

    randpos = self.np_random.uniform(low=-0.005, high=0.005,
                                       size=(self.n_dof,))
    randvel = self.np_random.uniform(low=-0.005, high=0.005,
                                       size=(self.n_dof,))
    for i, dof_idx in enumerate(self.dof_idx):
        p.resetJointState(self.draco, dof_idx, self.ini_pos[i] + randpos[i],
                          randvel[i])
    (_, _, self.initial_z), _ = p.getBasePositionAndOrientation(self.draco)
    obs = self.get_observation()

    return obs

  def render(self, mode='human', close=False):
      return

  # ============================================================================
  # Observation
  # ============================================================================
  def get_observation(self):
    j = np.array([self.get_relative_joint_position(jidx) for jidx in range(self.n_dof)] + [0.1*self.get_joint_veloicity(jidx) for jidx in range(self.n_dof)])
    self.n_joints_at_limit = np.count_nonzero(np.abs(j[0:self.n_dof]) > 0.99)
    self.joint_speed = j[self.n_dof:2*self.n_dof]

    (x, y, z), base_quat = p.getBasePositionAndOrientation(self.draco)
    (xdot, ydot, zdot), so3 = p.getBaseVelocity(self.draco)
    roll, pitch, yaw = p.getEulerFromQuaternion(base_quat)

    walk_target_theta = np.arctan2(self.target_y - y, self.target_x - x)
    self.walk_target_dist = np.linalg.norm([self.target_y - y, self.target_x - x])
    angle_to_target = walk_target_theta - yaw
    torso_rot = np.array([[np.cos(-yaw), -np.sin(-yaw), 0],
                          [np.sin(-yaw), np.cos(-yaw), 0],
                          [0, 0, 1]])
    xdot_local, ydot_local, zdot_local = np.dot(torso_rot, np.array([xdot, ydot, zdot]))
    more = np.array([z - self.initial_z, np.sin(angle_to_target), np.cos(angle_to_target),
                     0.3*xdot_local, 0.3*ydot_local, 0.3*zdot_local, roll, pitch], dtype=np.float32)
    b_contact, _ = self.get_contact_forces()

    return np.clip( np.concatenate([more] + [j] + [np.array(b_contact)]), -5, 5 )

  # ============================================================================
  # Kinematics Query
  # ============================================================================
  def get_joint_position(self, jidx):
    return p.getJointState(self.draco, jidx)[0]

  def get_joint_veloicity(self, jidx):
    return p.getJointState(self.draco, jidx)[1]

  def get_relative_joint_position(self, jidx):
    jp = self.get_joint_position(jidx)
    jp_mid = 0.5 * (self.dof_lb[jidx] + self.dof_ub[jidx])
    return 2 * (jp - jp_mid) / (self.dof_ub[jidx] - self.dof_lb[jidx])

  def get_link_com_pos(self, link_idx):
    return p.getLinkState(self.draco, link_idx,
                          computeLinkVelocity=False,
                          computeForwardKinematics=True)[0]

  def get_link_com_quat(self, link_idx):
    return p.getLinkState(self.draco, link_idx,
                          computeLinkVelocity=False,
                          computeForwardKinematics=True)[1]

  def get_link_local_com_pos(self, link_idx):
    return p.getLinkState(self.draco, link_idx,
                          computeLinkVelocity=False,
                          computeForwardKinematics=True)[2]

  def get_link_local_com_quat(self, link_idx):
    return p.getLinkState(self.draco, link_idx,
                          computeLinkVelocity=False,
                          computeForwardKinematics=True)[3]

  def get_link_pos(self, link_idx):
    return p.getLinkState(self.draco, link_idx,
                          computeLinkVelocity=False,
                          computeForwardKinematics=True)[4]

  def get_link_quat(self, link_idx):
    return p.getLinkState(self.draco, link_idx,
                          computeLinkVelocity=False,
                          computeForwardKinematics=True)[5]

  def get_link_vel(self, link_idx):
    return p.getLinkState(self.draco, link_idx,
                          computeLinkVelocity=True,
                          computeForwardKinematics=True)[6]

  def get_link_so3(self, link_idx):
    return p.getLinkState(self.draco, link_idx,
                          computeLinkVelocity=True,
                          computeForwardKinematics=True)[7]

  # ============================================================================
  # Contact Query
  # ============================================================================
  def get_contact_forces(self):
    b_contact = [False] * len(self.feet)
    contact_forces = [np.zeros(shape=(3, ))]*len(self.feet)

    for foot_idx, foot_name in enumerate(self.feet):
        contact_result = p.getContactPoints(bodyA=self.draco, linkIndexA=self.link_list[foot_name])
        n_contact = len(contact_result)
        if n_contact == 0:
            b_contact[foot_idx] = False
            contact_forces[foot_idx] = np.zeros(shape=(3,))
        else:
            b_contact[foot_idx] = True
            contact_forces[foot_idx] = np.array( [sum(contact_result[i][10] for i in range(n_contact)),
                                                  sum(contact_result[i][12] for i in range(n_contact)),
                                                  sum(contact_result[i][9] for i in range(n_contact))] )
    return b_contact, contact_forces

