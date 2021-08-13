import numpy as np


class Config(object):
    CONTROLLER_DT = 0.00125
    N_SUBSTEP = 1
    CAMERA_DT = 0.05
    KP = 0.
    KD = 0.

    INITIAL_POS_WORLD_TO_BASEJOINT = [0, 0, 1.5 - 0.757]
    INITIAL_QUAT_WORLD_TO_BASEJOINT = [0., 0., 0., 1.]

    PRINT_TIME = False
    PRINT_ROBOT_INFO = False
    VIDEO_RECORD = False
    RECORD_FREQ = 10
    SIMULATE_CAMERA = False

    KP, KD = dict(), dict()

    KP["neck_pitch"] = 20.
    KD["neck_pitch"] = 1.

    KP["r_hip_ie"] = 300
    KD["r_hip_ie"] = 4
    KP["l_hip_ie"] = 300
    KD["l_hip_ie"] = 4

    KP["r_hip_aa"] = 300
    KD["r_hip_aa"] = 4
    KP["l_hip_aa"] = 300
    KD["l_hip_aa"] = 4

    KP["r_hip_fe"] = 400
    KD["r_hip_fe"] = 15
    KP["l_hip_fe"] = 400
    KD["l_hip_fe"] = 15

    KP["r_knee_fe_jd"] = 400
    KD["r_knee_fe_jd"] = 12
    KP["l_knee_fe_jd"] = 400
    KD["l_knee_fe_jd"] = 12

    KP["r_ankle_fe"] = 150
    KD["r_ankle_fe"] = 2
    KP["l_ankle_fe"] = 150.
    KD["l_ankle_fe"] = 2

    KP["r_ankle_ie"] = 150.
    KD["r_ankle_ie"] = 2
    KP["l_ankle_ie"] = 150.
    KD["l_ankle_ie"] = 2

    KP["r_shoulder_fe"] = 50.
    KD["r_shoulder_fe"] = 1.
    KP["l_shoulder_fe"] = 50.
    KD["l_shoulder_fe"] = 1.

    KP["r_shoulder_aa"] = 50.
    KD["r_shoulder_aa"] = 1.
    KP["l_shoulder_aa"] = 50.
    KD["l_shoulder_aa"] = 1.

    KP["r_shoulder_ie"] = 50.
    KD["r_shoulder_ie"] = 1.
    KP["l_shoulder_ie"] = 50.
    KD["l_shoulder_ie"] = 1.

    KP["r_elbow_fe"] = 30.
    KD["r_elbow_fe"] = 0.2
    KP["l_elbow_fe"] = 30.
    KD["l_elbow_fe"] = 0.2

    KP["r_wrist_ps"] = 10.
    KD["r_wrist_ps"] = 0.2
    KP["l_wrist_ps"] = 10.
    KD["l_wrist_ps"] = 0.2

    KP["r_wrist_pitch"] = 10.
    KD["r_wrist_pitch"] = 0.2
    KP["l_wrist_pitch"] = 10.
    KD["l_wrist_pitch"] = 0.2
