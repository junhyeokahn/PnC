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
		# PnC Objects
		self.interface_ = A1Interface.A1Interface()
		self.sensor_data_ = A1Interface.A1SensorData()
		self.command_ = A1Interface.A1Command()
		self.mpc_ = ConvexMPC.ConvexMPC()
