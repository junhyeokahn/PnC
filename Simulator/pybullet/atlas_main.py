import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + '/Simulator/pybullet')
sys.path.append(cwd + '/build/lib')

import time, math
from collections import OrderedDict
import copy
import signal
import shutil

import cv2
import pybullet as p
import numpy as np

np.set_printoptions(precision=2)

from Config.Atlas.pybullet_simulation import Config
from utils import pybullet_util

import atlas_interface


def set_initial_config(robot, joint_id):
    # shoulder_x
    p.resetJointState(robot, joint_id["l_arm_shx"], -np.pi / 4, 0.)
    p.resetJointState(robot, joint_id["r_arm_shx"], np.pi / 4, 0.)
    # elbow_y
    p.resetJointState(robot, joint_id["l_arm_ely"], -np.pi / 2, 0.)
    p.resetJointState(robot, joint_id["r_arm_ely"], np.pi / 2, 0.)
    # elbow_x
    p.resetJointState(robot, joint_id["l_arm_elx"], -np.pi / 2, 0.)
    p.resetJointState(robot, joint_id["r_arm_elx"], -np.pi / 2, 0.)
    # hip_y
    p.resetJointState(robot, joint_id["l_leg_hpy"], -np.pi / 4, 0.)
    p.resetJointState(robot, joint_id["r_leg_hpy"], -np.pi / 4, 0.)
    # knee
    p.resetJointState(robot, joint_id["l_leg_kny"], np.pi / 2, 0.)
    p.resetJointState(robot, joint_id["r_leg_kny"], np.pi / 2, 0.)
    # ankle
    p.resetJointState(robot, joint_id["l_leg_aky"], -np.pi / 4, 0.)
    p.resetJointState(robot, joint_id["r_leg_aky"], -np.pi / 4, 0.)


def signal_handler(signal, frame):
    if Config.VIDEO_RECORD:
        pybullet_util.make_video(video_dir)
    p.disconnect()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":

    # Environment Setup
    p.connect(p.GUI)
    p.resetDebugVisualizerCamera(cameraDistance=1.5,
                                 cameraYaw=120,
                                 cameraPitch=-30,
                                 cameraTargetPosition=[1, 0.5, 1.5])
    p.setGravity(0, 0, -9.8)
    p.setPhysicsEngineParameter(fixedTimeStep=Config.CONTROLLER_DT,
                                numSubSteps=Config.N_SUBSTEP)
    if Config.VIDEO_RECORD:
        video_dir = 'video/atlas_pnc'
        if os.path.exists(video_dir):
            shutil.rmtree(video_dir)
        os.makedirs(video_dir)

    # Create Robot, Ground
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    robot = p.loadURDF(cwd + "/RobotModel/atlas/atlas.urdf",
                       Config.INITIAL_POS_WORLD_TO_BASEJOINT,
                       Config.INITIAL_QUAT_WORLD_TO_BASEJOINT)

    p.loadURDF(cwd + "/RobotModel/ground/plane.urdf", [0, 0, 0],
               useFixedBase=1)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    nq, nv, na, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom = pybullet_util.get_robot_config(
        robot, Config.INITIAL_POS_WORLD_TO_BASEJOINT,
        Config.INITIAL_QUAT_WORLD_TO_BASEJOINT, Config.PRINT_ROBOT_INFO)

    # Initial Config
    set_initial_config(robot, joint_id)

    # Link Damping
    p.changeDynamics(robot, -1, linearDamping=0., angularDamping=0.)
    pybullet_util.set_link_damping(robot, link_id.values(), 0., 0.)

    # Joint Friction
    pybullet_util.set_joint_friction(robot, joint_id, 0)

    # Construct Interface
    interface = atlas_interface.AtlasInterface()
    sensor_data = atlas_interface.AtlasSensorData()
    command = atlas_interface.AtlasCommand()

    # Run Sim
    t = 0
    dt = Config.CONTROLLER_DT
    count = 0

    while (1):
        # Get SensorData
        if Config.SIMULATE_CAMERA and count % (Config.CAMERA_DT /
                                               Config.CONTROLLER_DT) == 0:
            camera_img = pybullet_util.get_camera_image_from_link(
                robot, link_id['head'], 50, 10, 60., 0.1, 10)
        sensor_data_dict = pybullet_util.get_sensor_data(
            robot, joint_id, link_id, pos_basejoint_to_basecom,
            rot_basejoint_to_basecom)

        rf_height = pybullet_util.get_link_iso(robot, link_id['r_sole'])[2, 3]
        lf_height = pybullet_util.get_link_iso(robot, link_id['l_sole'])[2, 3]
        sensor_data_dict['b_rf_contact'] = True if rf_height <= 0.01 else False
        sensor_data_dict['b_lf_contact'] = True if lf_height <= 0.01 else False

        # Get Keyboard Event
        keys = p.getKeyboardEvents()
        if pybullet_util.is_key_triggered(keys, '8'):
            interface.interrupt_logic.b_interrupt_button_eight = True
        elif pybullet_util.is_key_triggered(keys, '5'):
            interface.interrupt_logic.b_interrupt_button_five = True
        elif pybullet_util.is_key_triggered(keys, '4'):
            interface.interrupt_logic.b_interrupt_button_four = True
        elif pybullet_util.is_key_triggered(keys, '2'):
            interface.interrupt_logic.b_interrupt_button_two = True
        elif pybullet_util.is_key_triggered(keys, '6'):
            interface.interrupt_logic.b_interrupt_button_six = True
        elif pybullet_util.is_key_triggered(keys, '7'):
            interface.interrupt_logic.b_interrupt_button_seven = True
        elif pybullet_util.is_key_triggered(keys, '9'):
            interface.interrupt_logic.b_interrupt_button_nine = True

        # Copy sensor_data_dict
        sensor_data.base_com_pos = sensor_data_dict["base_com_pos"]
        sensor_data.base_com_quat = np.array([
            sensor_data_dict["base_com_quat"][3],
            sensor_data_dict["base_com_quat"][0],
            sensor_data_dict["base_com_quat"][1],
            sensor_data_dict["base_com_quat"][2],
        ])
        sensor_data.base_com_lin_vel = sensor_data_dict["base_com_lin_vel"]
        sensor_data.base_com_ang_vel = sensor_data_dict["base_com_ang_vel"]
        sensor_data.base_joint_pos = sensor_data_dict["base_joint_pos"]
        sensor_data.base_joint_quat = np.array([
            sensor_data_dict["base_joint_quat"][3],
            sensor_data_dict["base_joint_quat"][0],
            sensor_data_dict["base_joint_quat"][1],
            sensor_data_dict["base_joint_quat"][2],
        ])
        sensor_data.base_joint_lin_vel = sensor_data_dict["base_joint_lin_vel"]
        sensor_data.base_joint_ang_vel = sensor_data_dict["base_joint_ang_vel"]
        sensor_data.joint_positions = sensor_data_dict["joint_pos"]
        sensor_data.joint_velocities = sensor_data_dict["joint_vel"]

        # Compute Command
        if Config.PRINT_TIME:
            start_time = time.time()
        interface.getCommand(sensor_data, command)

        if Config.PRINT_TIME:
            end_time = time.time()
            print("ctrl computation time: ", end_time - start_time)

        # Apply Trq
        pybullet_util.set_motor_trq(robot, joint_id, command.joint_torques)

        # Save Image
        if (Config.VIDEO_RECORD) and (count % Config.RECORD_FREQ == 0):
            frame = pybullet_util.get_camera_image([1.2, 0.5, 1.], 2.0, 120,
                                                   -15, 0, 60., 1920, 1080,
                                                   0.1, 100.)
            frame = frame[:, :, [2, 1, 0]]  # << RGB to BGR
            filename = video_dir + '/step%06d.jpg' % count
            cv2.imwrite(filename, frame)

        p.stepSimulation()

        # time.sleep(dt)
        t += dt
        count += 1
