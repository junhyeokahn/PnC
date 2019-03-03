import logging
import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import subprocess
import pybullet as p
import pybullet_data
from pkg_resources import parse_version
import os
import pybullet_data

PROJECT_PATH = os.getcwd()

logger = logging.getLogger(__name__)

class MyBulletHumanoidEnv(gym.Env):
  metadata = {
    'render.modes': ['human', 'rgb_array'],
    'video.frames_per_second' : 50
  }

  def __init__(self, render=False):
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
    # self.hanging_pos = [0, 0, 0.895]
    self.hanging_pos = [0, 0, 1.5]

    self.humanoid = p.loadMJCF(
            "/Users/junhyeokahn/Repository/bullet3/examples/pybullet/gym/pybullet_data/mjcf/humanoid_symmetric.xml")[0]

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    self.feet = ['l_foot', 'r_foot']

    self.joint_list = {}
    self.link_list = {}
    self.dof_idx = []
    dof_lb = []
    dof_ub = []
    dof_max_force = []
    active_joint = 0
    for j in range (p.getNumJoints(self.humanoid)):
        info = p.getJointInfo(self.humanoid, j)
        joint_name = info[1]
        joint_type = info[2]
        link_name = info[12]
        self.joint_list[joint_name.decode()] = j
        self.link_list[link_name.decode()] = j
        if (joint_type==p.JOINT_PRISMATIC or joint_type==p.JOINT_REVOLUTE):
            self.dof_idx.append(j)
            dof_lb.append(info[8])
            dof_ub.append(info[9])
            dof_max_force.append(info[10])
            active_joint+=1
    self.n_dof = active_joint

    # ==========================================================================
    # Observations \in R^{3 + 4 + dof + 3 + 3 + dof + dof + 3*2} :
    #    [base_pos, base_quat, q, base_vel, base_so3, qdot, actions, rfs]
    # Actions \in R^{n_dof}
    # ==========================================================================
    obs_high = np.array([np.inf] * (3+4+self.n_dof+3+3+self.n_dof+self.n_dof+3*2))
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
    # ==========================================================================
    # Apply action
    # ==========================================================================
    base_pos, base_quat, _, _, _, _ = self.get_state()
    pos_before = base_pos[0]
    clamped_action = np.clip(np.array(action), self.action_space.low,
                                               self.action_space.high)
    force = clamped_action * self.action_sclae

    p.setJointMotorControlArray(self.humanoid, self.dof_idx, p.TORQUE_CONTROL,
                                forces=force)
    p.stepSimulation()

    # ==========================================================================
    # Get observation
    # ==========================================================================
    base_pos, base_quat, q, base_vel, base_so3, qdot = self.get_state()
    pos_after = base_pos[0]
    base_euler = p.getEulerFromQuaternion(base_quat)

    # ==========================================================================
    # Termination condition
    # ==========================================================================
    done_info = [False] * 5
    if (np.abs(base_pos[2] - self.base_ini_pos[2]) > 0.10):
        done_info[0] = True
    if (np.abs(base_pos[1] - self.base_ini_pos[1]) > 0.20):
        done_info[1] = True
    if (np.abs(base_euler[0]) > 0.785):
        done_info[2] = True
    if (np.abs(base_euler[1]) > 0.785):
        done_info[3] = True
    if (np.abs(base_euler[2]) > 0.785):
        done_info[4] = True
    done = any(done_info)
    # print(done_info)

    # ==========================================================================
    # Reward
    # ==========================================================================
    vel_rew = 5.0 * (pos_after - pos_before) / self.timeStep
    alive_bonus = 5.0
    action_pen = 0.5 * np.square(clamped_action).sum()
    deviation_pen = 3 * np.square(base_pos[1:3]).sum()

    b_contact, contact_forces = self.get_contact_forces()

    impact_pen = 1.0 * sum( [np.linalg.norm(cf) for cf in contact_forces] ) / 380.0
    impact_pen = min(impact_pen, 3)

    jump_pen = 0.0;
    if not any(b_contact):
        jump_pen = -5.0

    ## TEST ##
    # reward = vel_rew + alive_bonus - action_pen - deviation_pen - impact_pen - jump_pen
    ## TEST ##
    reward = vel_rew + alive_bonus
    if done:
        reward = 0.0

    obs = base_pos + base_quat + q + base_vel + base_so3 + qdot + clamped_action.tolist()
    for i in range(len(b_contact)):
        obs = obs + contact_forces[i].tolist()

    # print("alive_bonus : {}, vel_rew : {}, action_pen : {}, deviation_pen : {}, impact_pen : {}, total_rew : {}".format(alive_bonus, vel_rew, action_pen, deviation_pen, impact_pen, reward))
    return np.array(obs), reward, done, {}

  def get_contact_forces(self):
    b_contact = [False] * len(self.feet)
    contact_forces = [np.zeros(shape=(3, ))]*len(self.feet)

    # for foot_idx, foot_name in enumerate(self.feet):
        # contact_result = p.getContactPoints(bodyA=self.humanoid, linkIndexA=self.link_list[foot_name])
        # n_contact = len(contact_result)
        # if n_contact == 0:
            # b_contact[foot_idx] = False
            # contact_forces[foot_idx] = np.zeros(shape=(3,))
        # else:
            # b_contact[foot_idx] = True
            # contact_forces[foot_idx] = np.array( [sum(contact_result[i][10] for i in range(n_contact)),
                                                  # sum(contact_result[i][12] for i in range(n_contact)),
                                                  # sum(contact_result[i][9] for i in range(n_contact))] )
    return b_contact, contact_forces

  def reset(self):
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    # self.humanoid = p.loadURDF(
            # PROJECT_PATH+"/RobotModel/Robot/Atlas/atlas_v4_with_multisense.urdf",
            # self.hanging_pos, useFixedBase=False)

    self.humanoid = p.loadMJCF(
            "/Users/junhyeokahn/Repository/bullet3/examples/pybullet/gym/pybullet_data/mjcf/humanoid_symmetric.xml")[0]
    p.changeDynamics(self.humanoid, -1, linearDamping=0, angularDamping=0)
    # self.timeStep = 0.01
    self.timeStep = 1./240.
    p.setJointMotorControlArray(self.humanoid, self.dof_idx, p.TORQUE_CONTROL,
                                forces=np.zeros(self.n_dof))
    p.setGravity(0,0, -9.81)
    p.setTimeStep(self.timeStep)
    p.setRealTimeSimulation(0)

    alpha = -np.pi/4.0
    beta = np.pi/5.5
    self.ini_pos = np.zeros(shape=(self.n_dof))
    randpos = self.np_random.uniform(low=-0.005, high=0.005,
                                       size=(self.n_dof,))
    randvel = self.np_random.uniform(low=-0.005, high=0.005,
                                       size=(self.n_dof,))
    for i, dof_idx in enumerate(self.dof_idx):
        p.resetJointState(self.humanoid, dof_idx, self.ini_pos[i] + randpos[i],
                          randvel[i])
    base_pos, base_quat, q, base_vel, base_so3, qdot = self.get_state()
    self.base_ini_pos = base_pos

    zero_actions = [0]*self.n_dof
    b_contact, contact_forces = self.get_contact_forces()
    obs = base_pos + base_quat + q + base_vel + base_so3 + qdot + zero_actions
    for i in range(len(b_contact)):
        obs = obs + contact_forces[i].tolist()

    return np.array(obs)

  def render(self, mode='human', close=False):
      return

  # ============================================================================
  # Joint State Query
  # ============================================================================
  def get_state(self):
    joint_states = p.getJointStates(self.humanoid, self.dof_idx)
    q = [state[0] for state in joint_states]
    qdot = [state[1] for state in joint_states]
    base_pos, base_quat, base_vel, base_so3 = (), (), (), ()
    base_pos, base_quat= p.getBasePositionAndOrientation(self.humanoid)
    base_vel, base_so3 = p.getBaseVelocity(self.humanoid)

    return list(base_pos), list(base_quat), q, \
           list(base_vel), list(base_so3), qdot


  # ============================================================================
  # Link State Query
  # ============================================================================
  def get_link_com_pos(self, link_idx):
    return p.getLinkState(self.humanoid, link_idx,
                          computeLinkVelocity=False,
                          computeForwardKinematics=True)[0]

  def get_link_com_quat(self, link_idx):
    return p.getLinkState(self.humanoid, link_idx,
                          computeLinkVelocity=False,
                          computeForwardKinematics=True)[1]

  def get_link_local_com_pos(self, link_idx):
    return p.getLinkState(self.humanoid, link_idx,
                          computeLinkVelocity=False,
                          computeForwardKinematics=True)[2]

  def get_link_local_com_quat(self, link_idx):
    return p.getLinkState(self.humanoid, link_idx,
                          computeLinkVelocity=False,
                          computeForwardKinematics=True)[3]

  def get_link_pos(self, link_idx):
    return p.getLinkState(self.humanoid, link_idx,
                          computeLinkVelocity=False,
                          computeForwardKinematics=True)[4]

  def get_link_quat(self, link_idx):
    return p.getLinkState(self.humanoid, link_idx,
                          computeLinkVelocity=False,
                          computeForwardKinematics=True)[5]

  def get_link_vel(self, link_idx):
    return p.getLinkState(self.humanoid, link_idx,
                          computeLinkVelocity=True,
                          computeForwardKinematics=True)[6]

  def get_link_so3(self, link_idx):
    return p.getLinkState(self.humanoid, link_idx,
                          computeLinkVelocity=True,
                          computeForwardKinematics=True)[7]
