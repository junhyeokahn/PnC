import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + '/utils/python_utils')
sys.path.append(cwd + '/simulator/pybullet')
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

from config.draco.pybullet_simulation import Config
import pybullet_util
import util

import draco_interface

if Config.B_USE_MESHCAT:
    from pinocchio.visualize import MeshcatVisualizer
    import pinocchio as pin


def set_initial_config(robot, joint_id):
    # Upperbody
    p.resetJointState(robot, joint_id["l_shoulder_aa"], np.pi / 6, 0.)
    p.resetJointState(robot, joint_id["l_elbow_fe"], -np.pi / 2, 0.)
    p.resetJointState(robot, joint_id["r_shoulder_aa"], -np.pi / 6, 0.)
    p.resetJointState(robot, joint_id["r_elbow_fe"], -np.pi / 2, 0.)

    # Lowerbody
    # hip_yaw_angle = 0.047
    hip_yaw_angle = 0.020
    p.resetJointState(robot, joint_id["l_hip_aa"], hip_yaw_angle, 0.)
    p.resetJointState(robot, joint_id["l_hip_fe"], -np.pi / 4, 0.)
    p.resetJointState(robot, joint_id["l_knee_fe_jp"], np.pi / 4, 0.)
    p.resetJointState(robot, joint_id["l_knee_fe_jd"], np.pi / 4, 0.)
    p.resetJointState(robot, joint_id["l_ankle_fe"], -np.pi / 4, 0.)
    p.resetJointState(robot, joint_id["l_ankle_ie"],
                      np.radians(-hip_yaw_angle), 0.)

    p.resetJointState(robot, joint_id["r_hip_aa"], -hip_yaw_angle, 0.)
    p.resetJointState(robot, joint_id["r_hip_fe"], -np.pi / 4, 0.)
    p.resetJointState(robot, joint_id["r_knee_fe_jp"], np.pi / 4, 0.)
    p.resetJointState(robot, joint_id["r_knee_fe_jd"], np.pi / 4, 0.)
    p.resetJointState(robot, joint_id["r_ankle_fe"], -np.pi / 4, 0.)
    p.resetJointState(robot, joint_id["r_ankle_ie"], hip_yaw_angle, 0.)


def signal_handler(signal, frame):
    if Config.VIDEO_RECORD:
        pybullet_util.make_video(video_dir, False)
    p.disconnect()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":

    # Environment Setup
    if Config.B_USE_MESHCAT:
        p.connect(p.DIRECT)
    else:
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(cameraDistance=2.5,
                                     cameraYaw=210,
                                     cameraPitch=-30,
                                     cameraTargetPosition=[0, 0, 0.5])
    p.setGravity(0, 0, -9.8)
    p.setPhysicsEngineParameter(fixedTimeStep=Config.CONTROLLER_DT,
                                numSubSteps=Config.N_SUBSTEP)
    if Config.VIDEO_RECORD:
        video_dir = 'video/draco'
        if os.path.exists(video_dir):
            shutil.rmtree(video_dir)
        os.makedirs(video_dir)

    # Create Robot, Ground
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    robot = p.loadURDF(
        cwd + "/robot_model/draco3/draco3_gripper_mesh_updated.urdf",
        Config.INITIAL_POS_WORLD_TO_BASEJOINT,
        Config.INITIAL_QUAT_WORLD_TO_BASEJOINT)

    p.loadURDF(cwd + "/robot_model/ground/plane.urdf", [0, 0, 0],
               useFixedBase=1)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    nq, nv, na, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom = pybullet_util.get_robot_config(
        robot, Config.INITIAL_POS_WORLD_TO_BASEJOINT,
        Config.INITIAL_QUAT_WORLD_TO_BASEJOINT, Config.PRINT_ROBOT_INFO)

    # Add Gear constraint
    c = p.createConstraint(robot,
                           link_id['l_knee_fe_lp'],
                           robot,
                           link_id['l_knee_fe_ld'],
                           jointType=p.JOINT_GEAR,
                           jointAxis=[0, 1, 0],
                           parentFramePosition=[0, 0, 0],
                           childFramePosition=[0, 0, 0])
    p.changeConstraint(c, gearRatio=-1, maxForce=500, erp=10)

    c = p.createConstraint(robot,
                           link_id['r_knee_fe_lp'],
                           robot,
                           link_id['r_knee_fe_ld'],
                           jointType=p.JOINT_GEAR,
                           jointAxis=[0, 1, 0],
                           parentFramePosition=[0, 0, 0],
                           childFramePosition=[0, 0, 0])
    p.changeConstraint(c, gearRatio=-1, maxForce=500, erp=10)

    if Config.B_USE_MESHCAT:
        # Create Robot for Meshcat Visualization
        model, collision_model, visual_model = pin.buildModelsFromUrdf(
            cwd + "/robot_model/draco/draco.urdf", cwd + "/robot_model/draco",
            pin.JointModelFreeFlyer())
        viz = MeshcatVisualizer(model, collision_model, visual_model)
        try:
            viz.initViewer(open=True)
        except ImportError as err:
            print(
                "Error while initializing the viewer. It seems you should install Python meshcat"
            )
            print(err)
            sys.exit(0)
        viz.loadViewerModel()
        vis_q = pin.neutral(model)

    # Initial Config
    set_initial_config(robot, joint_id)

    # Link Damping
    pybullet_util.set_link_damping(robot, link_id.values(), 0., 0.)

    # Joint Friction
    pybullet_util.set_joint_friction(robot, joint_id, 0)

    # Construct Interface
    interface = draco_interface.DracoInterface()
    sensor_data = draco_interface.DracoSensorData()
    command = draco_interface.DracoCommand()

    # Run Sim
    t = 0
    dt = Config.CONTROLLER_DT
    count = 0
    jpg_count = 0

    nominal_sensor_data = pybullet_util.get_sensor_data(
        robot, joint_id, link_id, pos_basejoint_to_basecom,
        rot_basejoint_to_basecom)

    # pybullet_util.draw_link_frame(robot, link_id["l_hand_contact"], text="lh")
    # pybullet_util.draw_link_frame(robot, link_id["r_hand_contact"], text="rh")
    while (1):

        # while_start = time.time()

        # Get SensorData
        if Config.SIMULATE_CAMERA and count % (Config.CAMERA_DT /
                                               Config.CONTROLLER_DT) == 0:
            pass
        sensor_data_dict = pybullet_util.get_sensor_data(
            robot, joint_id, link_id, pos_basejoint_to_basecom,
            rot_basejoint_to_basecom)

        rf_height = pybullet_util.get_link_iso(robot,
                                               link_id['r_foot_contact'])[2, 3]
        lf_height = pybullet_util.get_link_iso(robot,
                                               link_id['l_foot_contact'])[2, 3]
        sensor_data_dict[
            'b_rf_contact'] = True if rf_height <= 0.005 else False
        sensor_data_dict[
            'b_lf_contact'] = True if lf_height <= 0.005 else False

        sensor_data_dict['imu_frame_iso'] = pybullet_util.get_link_iso(
            robot, link_id['torso_imu'])
        sensor_data_dict['imu_frame_vel'] = pybullet_util.get_link_vel(
            robot, link_id['torso_imu'])

        # Get Keyboard Event
        keys = p.getKeyboardEvents()
        if pybullet_util.is_key_triggered(keys, 'w'):
            interface.interrupt.b_interrupt_button_w = True
        elif pybullet_util.is_key_triggered(keys, 'x'):
            interface.interrupt.b_interrupt_button_x = True
        elif pybullet_util.is_key_triggered(keys, 'a'):
            interface.interrupt.b_interrupt_button_a = True
        elif pybullet_util.is_key_triggered(keys, 's'):
            interface.interrupt.b_interrupt_button_s = True
        elif pybullet_util.is_key_triggered(keys, 'd'):
            interface.interrupt.b_interrupt_button_d = True
        elif pybullet_util.is_key_triggered(keys, 'q'):
            interface.interrupt.b_interrupt_button_q = True
        elif pybullet_util.is_key_triggered(keys, 'e'):
            interface.interrupt.b_interrupt_button_e = True
        elif pybullet_util.is_key_triggered(keys, 'r'):
            interface.interrupt.b_interrupt_button_r = True
        elif pybullet_util.is_key_triggered(keys, 'f'):
            interface.interrupt.b_interrupt_button_f = True
        elif pybullet_util.is_key_triggered(keys, 'j'):
            interface.interrupt.b_interrupt_button_j = True
        elif pybullet_util.is_key_triggered(keys, 'k'):
            interface.interrupt.b_interrupt_button_k = True
        elif pybullet_util.is_key_triggered(keys, 'p'):
            force = [500, 0, 0]
            pos = [0, 0, 0]
            p.applyExternalForce(robot, -1, force, pos, p.WORLD_FRAME)
        elif pybullet_util.is_key_triggered(keys, 'z'):
            interface.interrupt.b_interrupt_button_z = True
        elif pybullet_util.is_key_triggered(keys, 'c'):
            interface.interrupt.b_interrupt_button_c = True

        # Copy sensor_data_dict
        sensor_data.imu_frame_iso = sensor_data_dict['imu_frame_iso']
        sensor_data.imu_frame_vel = sensor_data_dict['imu_frame_vel']
        sensor_data.joint_positions = sensor_data_dict["joint_pos"]
        sensor_data.joint_velocities = sensor_data_dict["joint_vel"]
        sensor_data.b_rf_contact = sensor_data_dict["b_rf_contact"]
        sensor_data.b_lf_contact = sensor_data_dict["b_lf_contact"]

        # TODO : Debugging purpose
        # Copy Base
        sensor_data.base_com_pos = sensor_data_dict['base_com_pos']
        qt = sensor_data_dict['base_com_quat']
        sensor_data.base_com_quat = np.array([qt[3], qt[0], qt[1], qt[2]])
        sensor_data.base_com_lin_vel = sensor_data_dict["base_com_lin_vel"]
        sensor_data.base_com_ang_vel = sensor_data_dict["base_com_ang_vel"]
        sensor_data.base_joint_pos = sensor_data_dict['base_joint_pos']
        sensor_data.base_joint_quat = np.array([qt[3], qt[0], qt[1], qt[2]])
        sensor_data.base_joint_lin_vel = sensor_data_dict["base_joint_lin_vel"]
        sensor_data.base_joint_ang_vel = sensor_data_dict["base_joint_ang_vel"]

        # Compute Command
        if Config.PRINT_TIME:
            start_time = time.time()
        interface.getCommand(sensor_data, command)

        command_joint_positions = copy.deepcopy(command.joint_positions)
        command_joint_velocities = copy.deepcopy(command.joint_velocities)
        command_joint_torques = copy.deepcopy(command.joint_torques)

        if Config.PRINT_TIME:
            end_time = time.time()
            print("ctrl computation time: ", end_time - start_time)

        # Exclude Knee Distal Joints Command
        del command_joint_positions["l_knee_fe_jp"]
        del command_joint_positions["r_knee_fe_jp"]
        del command_joint_velocities["l_knee_fe_jp"]
        del command_joint_velocities["r_knee_fe_jp"]
        del command_joint_torques["l_knee_fe_jp"]
        del command_joint_torques["r_knee_fe_jp"]

        # Apply Command
        pybullet_util.set_motor_trq(robot, joint_id, command_joint_torques)

        # Save Image
        if (Config.VIDEO_RECORD) and (count % Config.RECORD_FREQ == 0):
            frame = pybullet_util.get_camera_image([1., 0.5, 1.], 1.0, 120,
                                                   -15, 0, 60., 1920, 1080,
                                                   0.1, 100.)
            frame = frame[:, :, [2, 1, 0]]  # << RGB to BGR
            filename = video_dir + '/step%06d.jpg' % jpg_count
            cv2.imwrite(filename, frame)
            jpg_count += 1

        # if Config.B_USE_MESHCAT:
        # TODO: match idx to joint_pos
        # vis_q[0:3] = sensor_data_dict['base_joint_pos']
        # vis_q[3:7] = sensor_data_dict['base_joint_quat']
        # for i, (k, v) in enumerate(sensor_data_dict['joint_pos'].items()):
        # idx = interface._robot.get_q_idx(k)
        # vis_q[idx] = v
        # viz.display(vis_q)

        p.stepSimulation()
        # if count % 100 == 0:
        # __import__('ipdb').set_trace()

        time.sleep(dt)
        t += dt
        count += 1

        # while_end = time.time()
        # while_loop_time = while_end - while_start
        # print("while_loop_time:", while_loop_time)
        # exit()
