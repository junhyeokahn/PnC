import numpy as np


class Config(object):
    CONTROLLER_DT = 0.01
    N_SUBSTEP = 10
    # CONTROLLER_DT = 0.001
    # N_SUBSTEP = 1
    CAMERA_DT = 0.05
    KP = 0.
    KD = 0.

    INITIAL_POS_WORLD_TO_BASEJOINT = [0, 0, 1.5 - 0.761]
    INITIAL_QUAT_WORLD_TO_BASEJOINT = [0., 0., 0., 1.]

    PRINT_TIME = False
    PRINT_ROBOT_INFO = False
    VIDEO_RECORD = False
    RECORD_FREQ = 10
    SIMULATE_CAMERA = False
