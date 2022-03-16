####################################
'''
A script for running the real A1 robot with WBIC + MPC
Author: Ryan Gupta
'''
####################################


# Basic Packages
import time
import os
import sys
import pickle 
import getch
import argparse
import copy
import random
import collections
import cv2
# Other Packages
from multiprocessing import Process, Queue
from multiprocessing.managers import BaseManager
from queue import LifoQueue
import threading 
import numpy as np
from math import exp
# My packages
sys.path.append('/home/knapsack/project/perceptive_locomotion/perceptive-locomotion')
sys.path.append('/home/knapsack/project/perceptive_locomotion/perceptive-locomotion/src')
sys.path.append('/home/knapsack/project/locomotion/src/PnC/build/lib')
# IMU
from vn100 import *
# Interface with real A1
from robot_interface import RobotInterface
# Initialization (Stand Up)
from real_robot_init import RealRobotInit 
# PnC
import A1Interface
import ConvexMPC

'''
Global Vars
'''
PATH_SRC    = os.path.dirname(os.path.realpath(__file__))
PATH_ROOT   = os.path.dirname("/home/knapsack/project/locomotion/src/PnC")
# PATH_DATA   = PATH_ROOT+"/data"
# TODO: 
PATH_SAVE   = PATH_ROOT+"/ExperimentData"

NUM_LEGS=4
NUM_LEG_MOTORS=3

JOINT_POS_ZERO = np.array([0, 0.9, -1.8]*NUM_LEGS)
JOINT_VEL_ZERO = np.array([0]*NUM_LEGS*NUM_LEG_MOTORS)

def quat_to_rot_matrix(quat):
    q0 = quat[0][0]
    q1 = quat[0][1]
    q2 = quat[0][2]
    q3 = quat[0][3]

    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
    return rot_matrix

# create manager that knows how to create and manage LifoQueues
class MyManager(BaseManager):
    pass
MyManager.register('LifoQueue', LifoQueue)

class envQuadHW():
    def __init__(self):
        # Message queues for thread info exchange
        self.rxn_buffer = Queue()
        # TODO: manage the max size of this
        manager = MyManager()
        manager.start()
        self.fdb_buffer = manager.LifoQueue()

        # PnC Objects
        self.interface_ = A1Interface.A1Interface()
        self.sensor_data_ = A1Interface.A1SensorData()
        self.command_ = A1Interface.A1Command()

        # MPC Constructor Inputs
        mass = float(11.7)
        num_legs = int(4)
        body_inertia = (0.07335, 0.0, 0.0, 0.0, 0.25068, 0.0, 0.0, 0.0, 0.25447)
        _MPC_WEIGHTS = (5., 5., 0.2, 0., 0., 50, 0.5, 0.5, 0.2, 10., 5., 0.1, 0.)
        _PLANNING_HORIZON_STEPS = int(10)
        _PLANNING_TIMESTEP = float(0.025)
        qp_solver = ConvexMPC.OSQP
        
        # ConvexMPC
        self.mpc_ = ConvexMPC.ConvexMPC(mass, body_inertia, NUM_LEGS,
                                          _PLANNING_HORIZON_STEPS,
                                          _PLANNING_TIMESTEP,
                                          _MPC_WEIGHTS, 1e-5,
                                          qp_solver)

        # Multiprocessing
        self.thread_cmd = Process(target=self._get_targets)
        self.thread_mpc = Process(target=self._run_mpc)

        # Loop Rates
        self.control_step = 0.002 # 500Hz
        self.mpc_step = 0.02 # 50 Hz

        # MPC feedback
        #   16 + 12 + 16 (all but state_progression_ and des_states_)
        self.feedback = np.zeros(44)
        # Velocity command
        self.command = np.zeros(3, dtype=np.float32)

        self.operation_flag = True

        # Real Robot Interface
        self.interface = RobotInterface()
        # PD Gains for robot init
        self.kp = np.array([130 , 110 , 140 , 130 , 110 , 140 , 130, 110, 140 , 130 , 110, 140])*0.7
        self.kd = np.array([0.5, 0.3, 0.2, 0.5, 0.3, 0.2, 0.5, 0.3, 0.2, 0.5, 0.3, 0.2])*8
        # Initialize Real Robot
        self.robot_init = RealRobotInit(self.interface, self.kp, self.kd)
        self.robot_init.get_init_config()
        # Safety Confirmation
        print("robot_init.starting_config = ", self.robot_init.starting_config)
        print("Ensure starting config not zero vector")
        print("Then press any key ...")
        getch.getch()
        # Init robot
        self.robot_init.init_motion()
        self.robot_init.hold_pose(2)

    def reset(self):
        self._setup()

    def _setup(self):

        self.robot_init.get_init_config()
        self.robot_init.set_desired_config(JOINT_POS_ZERO)

        self.robot_init.init_motion()
        self.robot_init.hold_pose(2)

        self.policy_time = time.time()
        self.feedback_time = time.time()
        self.control_time = time.time()
        self.command_time = time.time()

        self.thread_cmd.start()
        self.thread_mpc.start()

    def _run_mpc(self):

        state_progression_ = np.zeros(13 * _PLANNING_HORIZON_STEPS)
        des_states_ = np.zeros(13 * _PLANNING_HORIZON_STEPS)

        # foot_friction_coeffs = np.array([0.6, 0.6, 0.6, 0.6])
        # flfoot_body_frame = np.zeros(3)
        # frfoot_body_frame = np.zeros(3)
        # rloot_body_frame = np.zeros(3)
        # rrfoot_body_frame = np.zeros(3)

        while self.operation_flag:
            if self.fdb_buffer.qsize():
                self.feedback = self.fdb_buffer.get()

            # # self.feedback len 44
            # # TODO is this the correct return val?
            contact_forces = self.mpc_.ComputeContactForces(
                    self.feedback[0:3], # com pos
                    self.feedback[3:6], # com vel
                    self.feedback[6:9], # com rpy
                    self.feedback[9:12], # com ang vel
                    self.feedback[12:16], # foot contact
                    self.feedback[16:28], # foot pos body frame
                    self.feedback[28:32], # friction coeffs
                    self.feedback[32:35], # com pos des
                    self.feedback[35:38], # com vel des
                    self.feedback[38:41], # com rpy des
                    self.feedback[41:], # com ang vel des
                    state_progression_,
                    des_states_)
            # TODO: what is size/shape and how to we handle this in A1CtrlArch
            # self.rxn_buffer.put(contact_forces)
