####################################
'''
A script for running the real A1 robot with WBIC + MPC
Author: Ryan Gupta
'''
####################################


# Basic Packages
import threading 
import time
import os
import sys
# Data mgmt
import queue
import pickle 
import getch
import copy
import collections

from math import exp
import numpy as np
import random

import argparse
import yaml

# My packages
sys.path.append('/home/mingyo/Projects/perceptive_locomotion/perceptive-locomotion')
sys.path.append('/home/mingyo/Projects/perceptive_locomotion/perceptive-locomotion/src')
sys.path.append('/home/mingyo/Projects/PnC/build/lib')
# IMU
from vn100 import *
# Interface with real A1
from robot_interface import RobotInterface
# Initialization (Stand Up)
from real_robot_init import RealRobotInit 
# PnC
import A1Interface
# import ConvexMPC

'''
Global Vars
'''
PATH_SRC    = os.path.dirname(os.path.realpath(__file__))
PATH_ROOT   = os.path.dirname("/home/mingyo/Projects/PnC")
# PATH_DATA   = PATH_ROOT+"/data"
# TODO: 
PATH_SAVE   = PATH_ROOT+"/Config/save"

NUM_LEGS=4
NUM_LEG_MOTORS=3

JOINT_POS_ZERO = np.array([0, 0.9, -1.8]*NUM_LEGS)
JOINT_VEL_ZERO = np.array([0]*NUM_LEGS*NUM_LEG_MOTORS)


class envQuadHW():
    def __init__(self):
        # Message queues for thread info exchange
        self.rxn_buffer = queue.Queue()
        # TODO: manage the max size of this
        self.fdb_buffer = queue.LifoQueue()

        # WBIC Objects
        self.interface_ = A1Interface.A1Interface()
        self.sensor_data_ = A1Interface.A1SensorData()
        self.command_ = A1Interface.A1Command()

        # MPC Params
        body_inertia = np.zeros(9);
        _MPC_WEIGHTS = np.zeros(13);
        body_inertia[0] = 0.01; body_inertia[4] = 0.03; body_inertia[8] = 0.03;
        _MPC_WEIGHTS[0] = 5.; _MPC_WEIGHTS[1] = 5.; _MPC_WEIGHTS[2] = 0.2;
        _MPC_WEIGHTS[5] = 50.; _MPC_WEIGHTS[6] = 0.5; _MPC_WEIGHTS[7] = 0.5;
        _MPC_WEIGHTS[8] = 0.2; _MPC_WEIGHTS[9] = 10; _MPC_WEIGHTS[10] = 5.;
        _MPC_WEIGHTS[11] = 0.1; 
        mass = 11.7
        _PLANNING_HORIZON_STEPS = 6;
        _PLANNING_TIMESTEP = 0.04
        
        # ConvexMPC
        # self.mpc_ = ConvexMPC.ConvexMPC(mass, body_inertia, NUM_LEGS,
        #                                   _PLANNING_HORIZON_STEPS,
        #                                   _PLANNING_TIMESTEP,
        #                                   _MPC_WEIGHTS)

        # Multithreading
        self.thread_cmd = threading.Thread(target=self._get_targets)
        self.thread_mpc = threading.Thread(target=self._run_mpc)

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

        while self.operation_flag:
            if self.fdb_buffer.qsize():
                self.feedback = self.fdb_buffer.get()

            # # self.feedback len 44
            # # TODO is this the correct return val?
            # contact_forces = self.mpc_.ComputeContactForces(
                                #     self.feedback[0:3], # com pos
                                #     self.feedback[3:6], # com vel
                                #     self.feedback[6:9], # com rpy
                                #     self.feedback[9:12], # com ang vel
                                #     self.feedback[12:16], # foot contact
                                #     self.feedback[16:28], # foot pos body frame
                                #     self.feedback[28:32], # friction coeffs
                                #     self.feedback[32:35], # com pos des
                                #     self.feedback[35:38], # com vel des
                                #     self.feedback[38:41], # com rpy des
                                #     self.feedback[41:], # com ang vel des
                                #     state_progression_,
                                #     des_states_)
            # TODO: what is size/shape and how to we handle this in A1CtrlArch
            # self.rxn_buffer.put(contact_forces)
