import os
PROJECT_PATH = os.getcwd()

import pybullet
import pybullet_data
from pybullet_envs.robot_bases import URDFBasedRobot
from pybullet_envs.gym_locomotion_envs import WalkerBaseBulletEnv

import numpy as np

class WalkerBase2(URDFBasedRobot):
	def __init__(self,  fn, robot_name, action_dim, obs_dim, base_pos, base_ori, power):
		URDFBasedRobot.__init__(self, fn, robot_name, action_dim, obs_dim, base_pos, base_ori)
		self.power = power
		self.camera_x = 0
		self.start_pos_x, self.start_pos_y, self.start_pos_z = 0, 0, 0
		self.walk_target_x = 1e3  # kilometer away
		self.walk_target_y = 0
		self.body_xyz=[0,0,0]

	def robot_specific_reset(self, bullet_client):
		self._p = bullet_client
		for j in self.ordered_joints:
			j.reset_current_position(self.np_random.uniform(low=-0.1, high=0.1), 0)

		self.feet = [self.parts[f] for f in self.foot_list]
		self.feet_contact = np.array([0.0 for f in self.foot_list], dtype=np.float32)
		self.scene.actor_introduce(self)
		self.initial_z = None

	def apply_action(self, a):
		assert (np.isfinite(a).all())
		for n, j in enumerate(self.ordered_joints):
			j.set_motor_torque(self.power * j.power_coef * float(np.clip(a[n], -1, +1)))

	def calc_state(self):
		j = np.array([j.current_relative_position() for j in self.ordered_joints], dtype=np.float32).flatten()
		# even elements [0::2] position, scaled to -1..+1 between limits
		# odd elements  [1::2] angular speed, scaled to show -1..+1
		self.joint_speeds = j[1::2]
		self.joints_at_limit = np.count_nonzero(np.abs(j[0::2]) > 0.99)

		body_pose = self.robot_body.pose()
		parts_xyz = np.array([p.pose().xyz() for p in self.parts.values()]).flatten()
		self.body_xyz = (
		parts_xyz[0::3].mean(), parts_xyz[1::3].mean(), body_pose.xyz()[2])  # torso z is more informative than mean z
		self.body_rpy = body_pose.rpy()
		z = self.body_xyz[2]
		if self.initial_z == None:
			self.initial_z = z
		r, p, yaw = self.body_rpy
		self.walk_target_theta = np.arctan2(self.walk_target_y - self.body_xyz[1],
											self.walk_target_x - self.body_xyz[0])
		self.walk_target_dist = np.linalg.norm(
			[self.walk_target_y - self.body_xyz[1], self.walk_target_x - self.body_xyz[0]])
		angle_to_target = self.walk_target_theta - yaw

		rot_speed = np.array(
			[[np.cos(-yaw), -np.sin(-yaw), 0],
			 [np.sin(-yaw), np.cos(-yaw), 0],
			 [		0,			 0, 1]]
		)
		vx, vy, vz = np.dot(rot_speed, self.robot_body.speed())  # rotate speed back to body point of view

		more = np.array([ z-self.initial_z,
			np.sin(angle_to_target), np.cos(angle_to_target),
			0.3* vx , 0.3* vy , 0.3* vz ,  # 0.3 is just scaling typical speed into -1..+1, no physical sense here
			r, p], dtype=np.float32)
		return np.clip( np.concatenate([more] + [j] + [self.feet_contact]), -5, +5)

class Draco(WalkerBase2):
	self_collision = False
	foot_list = ["lAnkle", "rAnkle"]

	def __init__(self):

		base_pos = [0, 0, 1.15]
		# base_ori = [0, 0, 0, 1]
		base_ori = pybullet.getQuaternionFromEuler([0, np.deg2rad(10), 0])
		# self.n_ac = 10
		# self.n_obs = 8+2*10+2
		self.n_ac = 8
		self.n_obs = 8+2*self.n_ac+2
		WalkerBase2.__init__(self,  PROJECT_PATH+"/RobotModel/Robot/Draco/FixedDracoSim_PyBullet.urdf", 'Torso', action_dim=self.n_ac, obs_dim=self.n_obs, base_pos=base_pos, base_ori=base_ori, power=1)

	def robot_specific_reset(self, bullet_client):
		WalkerBase2.robot_specific_reset(self, bullet_client)
       #          self.motor_names = ["lHipYaw", "lHipRoll", "lHipPitch", "lKnee", "lAnkle"]
		# self.motor_power  = [50, 100, 200, 200, 5.]
		# self.motor_names = ["rHipYaw", "rHipRoll", "rHipPitch", "rKnee", "rAnkle"]
		# self.motor_power  = [50, 100, 200, 200, 5.]
		self.motor_names = ["lHipYaw", "lHipRoll", "lHipPitch", "lKnee"]
		self.motor_power  = [50, 50, 200, 200]
		self.motor_names = ["rHipYaw", "rHipRoll", "rHipPitch", "rKnee"]
		self.motor_power  = [50, 50, 200, 200]


		alpha = -np.pi / 5.
		beta = np.pi / 4.
		# self.jdict['lHipPitch'].reset_current_position(alpha, 0)
		# self.jdict['rHipPitch'].reset_current_position(alpha, 0)
		# self.jdict['lKnee'].reset_current_position(beta-alpha, 0)
		# self.jdict['rKnee'].reset_current_position(beta-alpha, 0)
		# self.jdict['lAnkle'].reset_current_position(np.pi/2. - beta, 0)
		# self.jdict['rAnkle'].reset_current_position(np.pi/2. - beta, 0)
		# self.jdict['lAnkle'].reset_current_position(np.pi/2., 0)
		# self.jdict['rAnkle'].reset_current_position(np.pi/2., 0)
		self.jdict['lHipPitch'].reset_current_position(np.deg2rad(-10), 0)
		self.jdict['rHipPitch'].reset_current_position(np.deg2rad(-10), 0)
		self.jdict['lKnee'].reset_current_position(0.52, 0)
		self.jdict['rKnee'].reset_current_position(0.52, 0)

		self.motors = [self.jdict[n] for n in self.motor_names]
		if self.random_yaw:
			position = [0,0,0]
			orientation = [0,0,0]
			yaw = self.np_random.uniform(low=-3.14, high=3.14)
			if self.random_lean and self.np_random.randint(2)==0:
				cpose.set_xyz(0, 0, 1.4)
				if self.np_random.randint(2)==0:
					pitch = np.pi/2
					position = [0, 0, 0.45]
				else:
					pitch = np.pi*3/2
					position = [0, 0, 0.25]
				roll = 0
				orientation = [roll, pitch, yaw]
			else:
				position = [0, 0, 1.4]
				orientation = [0, 0, yaw]  # just face random direction, but stay straight otherwise
			self.robot_body.reset_position(position)
			self.robot_body.reset_orientation(orientation)
		self.initial_z = 0.8

	random_yaw = False
	random_lean = False

	def apply_action(self, a):
		assert( np.isfinite(a).all() )
		force_gain = 1
		for i, m, power in zip(range(self.n_ac), self.motors, self.motor_power):
			m.set_motor_torque(float(force_gain * power * self.power * np.clip(a[i], -1, +1)))

	def alive_bonus(self, z, pitch):
		# return +2 if (z > 0.75 and np.abs(pitch) < np.deg2rad(60))  else -1
		return +2 if z > 0.75  else -1

class Atlas(WalkerBase2):
	self_collision = False
	foot_list = ["l_foot", "r_foot"]

	def __init__(self):

		base_pos = [0, 0, 1.05]
		base_ori = [0, 0, 0, 1]
		self.n_ac = 30
		self.n_obs = 8+2*30+2
		WalkerBase2.__init__(self,  PROJECT_PATH+"/RobotModel/Robot/Atlas/atlas_v4_with_multisense.urdf", 'pelvis', action_dim=self.n_ac, obs_dim=self.n_obs, base_pos=base_pos, base_ori=base_ori, power=1)

	def robot_specific_reset(self, bullet_client):
		WalkerBase2.robot_specific_reset(self, bullet_client)
		self.motor_names = ['back_bkz', 'back_bky', 'back_bkx', 'l_arm_shz', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_wry', 'l_arm_wrx', 'l_arm_wry2', 'neck_ry', 'r_arm_shz', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_wry', 'r_arm_wrx', 'r_arm_wry2', 'l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky', 'l_leg_akx', 'r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', 'r_leg_akx']
		self.motor_power  = [106.0, 445.0, 300.0, 87.0, 99.0, 63.0, 112.0, 25.0, 25.0, 5.0, 5.0, 87.0, 99.0, 63.0, 112.0, 25.0, 25.0, 5.0, 275.0, 530.0, 840.0, 890.0, 92.0, 45.0, 275.0, 530.0, 840.0, 890.0, 92.0, 45.0]

		self.motors = [self.jdict[n] for n in self.motor_names]
		if self.random_yaw:
			position = [0,0,0]
			orientation = [0,0,0]
			yaw = self.np_random.uniform(low=-3.14, high=3.14)
			if self.random_lean and self.np_random.randint(2)==0:
				cpose.set_xyz(0, 0, 1.4)
				if self.np_random.randint(2)==0:
					pitch = np.pi/2
					position = [0, 0, 0.45]
				else:
					pitch = np.pi*3/2
					position = [0, 0, 0.25]
				roll = 0
				orientation = [roll, pitch, yaw]
			else:
				position = [0, 0, 1.4]
				orientation = [0, 0, yaw]  # just face random direction, but stay straight otherwise
			self.robot_body.reset_position(position)
			self.robot_body.reset_orientation(orientation)
		self.initial_z = 0.8

	random_yaw = False
	random_lean = False

	def apply_action(self, a):
		assert( np.isfinite(a).all() )
		force_gain = 1
		for i, m, power in zip(range(self.n_ac), self.motors, self.motor_power):
			m.set_motor_torque(float(force_gain * power * self.power * np.clip(a[i], -1, +1)))

	def alive_bonus(self, z, pitch):
		return +2 if z > 0.55 else -1

class DracoEnv2(WalkerBaseBulletEnv):
	def __init__(self, robot=Draco(), render=False):
		self.robot = robot
		WalkerBaseBulletEnv.__init__(self, self.robot, render)
		self.electricity_cost  = 4.25*WalkerBaseBulletEnv.electricity_cost
		self.stall_torque_cost = 1.0*WalkerBaseBulletEnv.stall_torque_cost

class AtlasEnv(WalkerBaseBulletEnv):
	def __init__(self, robot=Atlas(), render=False):
		self.robot = robot
		WalkerBaseBulletEnv.__init__(self, self.robot, render)
		self.electricity_cost  = 4.25*WalkerBaseBulletEnv.electricity_cost
		self.stall_torque_cost = 1.25*WalkerBaseBulletEnv.stall_torque_cost
