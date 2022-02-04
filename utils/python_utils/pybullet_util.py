import os
import sys
from collections import OrderedDict

import pybullet as p
import numpy as np
from tqdm import tqdm
import cv2
import imageio

from utils.python_utils import util
from utils.python_utils import liegroup


def get_robot_config(robot,
                     initial_pos=None,
                     initial_quat=None,
                     b_print_info=False):
    nq, nv, na, joint_id, link_id = 0, 0, 0, OrderedDict(), OrderedDict()
    link_id[(p.getBodyInfo(robot)[0]).decode("utf-8")] = -1
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        if info[2] != p.JOINT_FIXED:
            joint_id[info[1].decode("utf-8")] = info[0]
        link_id[info[12].decode("utf-8")] = info[0]
        nq = max(nq, info[3])
        nv = max(nv, info[4])
    nq += 1
    nv += 1
    na = len(joint_id)

    base_pos, base_quat = p.getBasePositionAndOrientation(robot)
    rot_world_com = util.quat_to_rot(base_quat)
    initial_pos = [0., 0., 0.] if initial_pos is None else initial_pos
    initial_quat = [0., 0., 0., 1.] if initial_quat is None else initial_quat
    rot_world_basejoint = util.quat_to_rot(np.array(initial_quat))
    pos_basejoint_to_basecom = np.dot(rot_world_basejoint.transpose(),
                                      base_pos - np.array(initial_pos))
    rot_basejoint_to_basecom = np.dot(rot_world_basejoint.transpose(),
                                      rot_world_com)

    if b_print_info:
        print("=" * 80)
        print("SimulationRobot")
        print("nq: ", nq, ", nv: ", nv, ", na: ", na)
        print("Vector from base joint frame to base com frame")
        print(pos_basejoint_to_basecom)
        print("Rotation from base joint frame to base com frame")
        print(rot_basejoint_to_basecom)
        print("+" * 80)
        print("Joint Infos")
        util.PrettyPrint(joint_id)
        print("+" * 80)
        print("Link Infos")
        util.PrettyPrint(link_id)

    return nq, nv, na, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom


def get_kinematics_config(robot, joint_id, link_id, open_chain_joints,
                          base_link, ee_link):
    joint_screws_in_ee = np.zeros((6, len(open_chain_joints)))
    ee_link_state = p.getLinkState(robot, link_id[ee_link], 1, 1)
    if link_id[base_link] == -1:
        base_pos, base_quat = p.getBasePositionAndOrientation(robot)
    else:
        base_link_state = p.getLinkState(robot, link_id[base_link], 1, 1)
        base_pos, base_quat = base_link_state[0], base_link_state[1]
    T_w_b = liegroup.RpToTrans(util.quat_to_rot(np.array(base_quat)),
                               np.array(base_pos))
    T_w_ee = liegroup.RpToTrans(util.quat_to_rot(np.array(ee_link_state[1])),
                                np.array(ee_link_state[0]))
    T_b_ee = np.dot(liegroup.TransInv(T_w_b), T_w_ee)
    for i, joint_name in enumerate(open_chain_joints):
        joint_info = p.getJointInfo(robot, joint_id[joint_name])
        link_name = joint_info[12].decode("utf-8")
        joint_type = joint_info[2]
        joint_axis = joint_info[13]
        screw_at_joint = np.zeros(6)
        link_state = p.getLinkState(robot, link_id[link_name], 1, 1)
        T_w_j = liegroup.RpToTrans(util.quat_to_rot(np.array(link_state[5])),
                                   np.array(link_state[4]))
        T_ee_j = np.dot(liegroup.TransInv(T_w_ee), T_w_j)
        Adj_ee_j = liegroup.Adjoint(T_ee_j)
        if joint_type == p.JOINT_REVOLUTE:
            screw_at_joint[0:3] = np.array(joint_axis)
        elif joint_type == p.JOINT_PRISMATIC:
            screw_at_joint[3:6] = np.array(joint_axis)
        else:
            raise ValueError
        joint_screws_in_ee[:, i] = np.dot(Adj_ee_j, screw_at_joint)

    return joint_screws_in_ee, T_b_ee


def get_link_iso(robot, link_idx):
    info = p.getLinkState(robot, link_idx, 1, 1)
    pos = np.array(info[0])
    rot = util.quat_to_rot(np.array(info[1]))

    return liegroup.RpToTrans(rot, pos)


def get_link_vel(robot, link_idx):
    info = p.getLinkState(robot, link_idx, 1, 1)
    ret = np.zeros(6)
    ret[3:6] = np.array(info[6])
    ret[0:3] = np.array(info[7])

    return ret


def set_link_damping(robot, link_id, lin_damping, ang_damping):
    for i in link_id:
        p.changeDynamics(robot,
                         i,
                         linearDamping=lin_damping,
                         angularDamping=ang_damping)


def set_joint_friction(robot, joint_id, max_force=0):
    p.setJointMotorControlArray(robot, [*joint_id.values()],
                                p.VELOCITY_CONTROL,
                                forces=[max_force] * len(joint_id))


def draw_link_frame(robot, link_idx, linewidth=5.0, text=None):
    # This only works when the link has an visual element defined in the urdf file
    if text is not None:
        p.addUserDebugText(text, [0, 0, 0.1],
                           textColorRGB=[1, 0, 0],
                           textSize=1.5,
                           parentObjectUniqueId=robot,
                           parentLinkIndex=link_idx)

    p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0],
                       linewidth,
                       parentObjectUniqueId=robot,
                       parentLinkIndex=link_idx)

    p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0],
                       linewidth,
                       parentObjectUniqueId=robot,
                       parentLinkIndex=link_idx)

    p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1],
                       linewidth,
                       parentObjectUniqueId=robot,
                       parentLinkIndex=link_idx)


def set_motor_impedance(robot, joint_id, command, kp, kd):
    trq_applied = OrderedDict()
    for (joint_name, pos_des), (_, vel_des), (_, trq_des) in zip(
            command['joint_pos'].items(), command['joint_vel'].items(),
            command['joint_trq'].items()):
        joint_state = p.getJointState(robot, joint_id[joint_name])
        joint_pos, joint_vel = joint_state[0], joint_state[1]
        trq_applied[joint_id[joint_name]] = trq_des + kp[joint_name] * (
            pos_des - joint_pos) + kd[joint_name] * (vel_des - joint_vel)

    p.setJointMotorControlArray(robot,
                                trq_applied.keys(),
                                controlMode=p.TORQUE_CONTROL,
                                forces=list(trq_applied.values()))


def set_motor_trq(robot, joint_id, trq_cmd):
    trq_applied = OrderedDict()
    for joint_name, trq_des in trq_cmd.items():
        trq_applied[joint_id[joint_name]] = trq_des

    p.setJointMotorControlArray(robot,
                                trq_applied.keys(),
                                controlMode=p.TORQUE_CONTROL,
                                forces=list(trq_applied.values()))


def set_motor_pos(robot, joint_id, pos_cmd):
    pos_applied = OrderedDict()
    for joint_name, pos_des in pos_cmd.items():
        pos_applied[joint_id[joint_name]] = pos_des

    p.setJointMotorControlArray(robot,
                                pos_applied.keys(),
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=list(pos_applied.values()))


def set_motor_pos_vel(robot, joint_id, pos_cmd, vel_cmd):
    pos_applied = OrderedDict()
    vel_applied = OrderedDict()
    for (joint_name, pos_des), (_, vel_des) in zip(pos_cmd.items(),
                                                   vel_cmd.items()):
        pos_applied[joint_id[joint_name]] = pos_des
        vel_applied[joint_id[joint_name]] = vel_des

    p.setJointMotorControlArray(robot,
                                pos_applied.keys(),
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=list(pos_applied.values()),
                                targetVelocities=list(vel_applied.values()))


def get_sensor_data(robot, joint_id, link_id, pos_basejoint_to_basecom,
                    rot_basejoint_to_basecom):
    """
    Parameters
    ----------
    joint_id (dict):
        Joint ID Dict
    link_id (dict):
        Link ID Dict
    pos_basejoint_to_basecom (np.ndarray):
        3d vector from base joint frame to base com frame
    rot_basejoint_to_basecom (np.ndarray):
        SO(3) from base joint frame to base com frame
    b_fixed_Base (bool);
        Whether the robot is floating or fixed
    Returns
    -------
    sensor_data (dict):
        base_com_pos (np.array):
            base com pos in world
        base_com_quat (np.array):
            base com quat in world
        base_com_lin_vel (np.array):
            base com lin vel in world
        base_com_ang_vel (np.array):
            base com ang vel in world
        base_joint_pos (np.array):
            base pos in world
        base_joint_quat (np.array):
            base quat in world
        base_joint_lin_vel (np.array):
            base lin vel in world
        base_joint_ang_vel (np.array):
            base ang vel in world
        joint_pos (dict):
            Joint pos
        joint_vel (dict):
            Joint vel
        b_rf_contact (bool):
            Right Foot Contact Switch
        b_lf_contact (bool):
            Left Foot Contact Switch
    """
    sensor_data = OrderedDict()

    # Handle Base Frame Quantities
    base_com_pos, base_com_quat = p.getBasePositionAndOrientation(robot)
    sensor_data['base_com_pos'] = np.asarray(base_com_pos)
    sensor_data['base_com_quat'] = np.asarray(base_com_quat)

    base_com_lin_vel, base_com_ang_vel = p.getBaseVelocity(robot)
    sensor_data['base_com_lin_vel'] = np.asarray(base_com_lin_vel)
    sensor_data['base_com_ang_vel'] = np.asarray(base_com_ang_vel)

    rot_world_com = util.quat_to_rot(np.copy(sensor_data['base_com_quat']))
    rot_world_joint = np.dot(rot_world_com,
                             rot_basejoint_to_basecom.transpose())
    sensor_data['base_joint_pos'] = sensor_data['base_com_pos'] - np.dot(
        rot_world_joint, pos_basejoint_to_basecom)
    sensor_data['base_joint_quat'] = util.rot_to_quat(rot_world_joint)
    trans_joint_com = liegroup.RpToTrans(rot_basejoint_to_basecom,
                                         pos_basejoint_to_basecom)
    adT_joint_com = liegroup.Adjoint(trans_joint_com)
    twist_com_in_world = np.zeros(6)
    twist_com_in_world[0:3] = np.copy(sensor_data['base_com_ang_vel'])
    twist_com_in_world[3:6] = np.copy(sensor_data['base_com_lin_vel'])
    augrot_com_world = np.zeros((6, 6))
    augrot_com_world[0:3, 0:3] = rot_world_com.transpose()
    augrot_com_world[3:6, 3:6] = rot_world_com.transpose()
    twist_com_in_com = np.dot(augrot_com_world, twist_com_in_world)
    twist_joint_in_joint = np.dot(adT_joint_com, twist_com_in_com)
    rot_world_joint = np.dot(rot_world_com,
                             rot_basejoint_to_basecom.transpose())
    augrot_world_joint = np.zeros((6, 6))
    augrot_world_joint[0:3, 0:3] = rot_world_joint
    augrot_world_joint[3:6, 3:6] = rot_world_joint
    twist_joint_in_world = np.dot(augrot_world_joint, twist_joint_in_joint)
    sensor_data['base_joint_lin_vel'] = np.copy(twist_joint_in_world[3:6])
    sensor_data['base_joint_ang_vel'] = np.copy(twist_joint_in_world[0:3])

    # Joint Quantities
    sensor_data['joint_pos'] = OrderedDict()
    sensor_data['joint_vel'] = OrderedDict()
    for k, v in joint_id.items():
        js = p.getJointState(robot, v)
        sensor_data['joint_pos'][k] = js[0]
        sensor_data['joint_vel'][k] = js[1]

    return sensor_data


def simulate_accelerometer_data(robot, link_id, previous_link_velocity, dt):
    gravity_vector = np.array([0., 0., -9.8])

    # calculate imu acceleration in world frame by numerical differentiation
    torso_acceleration = (get_link_vel(robot, link_id['torso_imu'])[3:6] - previous_link_velocity) / dt

    # map acceleration to IMU frame and add gravity constant
    imu_R_world = np.transpose(get_link_iso(robot, link_id['torso_imu']))[0:3, 0:3]
    accelerometer_measurement = np.dot(imu_R_world, (torso_acceleration + gravity_vector))

    return accelerometer_measurement


def add_sensor_noise(sensors_dictionary, noisy_sensors):
    # go through each sensor in the noisy_sensors dictionary
    for n_sensor in noisy_sensors:
        for element in range(len(sensors_dictionary[n_sensor])):
            noise = np.random.normal(0, noisy_sensors[n_sensor])

            # deal with the case where the sensor is a dictionary (e.g., joint positions)
            if (isinstance(sensors_dictionary[n_sensor], dict)):
                for val in sensors_dictionary[n_sensor]:
                    sensors_dictionary[n_sensor][val] += noise
            else:
                sensors_dictionary[n_sensor][element] += noise


def get_camera_image_from_link(robot, link, pic_width, pic_height, fov,
                               nearval, farval):
    aspect = pic_width / pic_height
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearval,
                                                     farval)
    link_info = p.getLinkState(robot, link, 1, 1)  #Get head link info
    link_pos = link_info[0]  #Get link com pos wrt world
    link_ori = link_info[1]  #Get link com ori wrt world
    rot = p.getMatrixFromQuaternion(link_ori)
    rot = np.array(rot).reshape(3, 3)

    global_camera_x_unit = np.array([1, 0, 0])
    global_camera_z_unit = np.array([0, 0, 1])

    camera_eye_pos = link_pos + np.dot(rot, 0.1 * global_camera_x_unit)
    camera_target_pos = link_pos + np.dot(rot, 1.0 * global_camera_x_unit)
    camera_up_vector = np.dot(rot, global_camera_z_unit)
    view_matrix = p.computeViewMatrix(camera_eye_pos, camera_target_pos,
                                      camera_up_vector)  #SE3_camera_to_world
    width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
        pic_width,  #image width
        pic_height,  #image height
        view_matrix,
        projection_matrix)
    return width, height, rgb_img, depth_img, seg_img, view_matrix, projection_matrix, camera_eye_pos


def make_video(video_dir, delete_jpgs=True):
    images = []
    for file in tqdm(sorted(os.listdir(video_dir)),
                     desc='converting jpgs to gif'):
        filename = video_dir + '/' + file
        im = cv2.imread(filename)
        im = im[:, :, [2, 1, 0]]  # << BGR to RGB
        images.append(im)
        if delete_jpgs:
            os.remove(filename)
    imageio.mimsave(video_dir + '/video.gif', images[:-1], duration=0.01)


def get_camera_image(cam_target_pos, cam_dist, cam_yaw, cam_pitch, cam_roll,
                     fov, render_width, render_height, nearval, farval):
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=cam_target_pos,
        distance=cam_dist,
        yaw=cam_yaw,
        pitch=cam_pitch,
        roll=cam_roll,
        upAxisIndex=2)
    proj_matrix = p.computeProjectionMatrixFOV(fov=fov,
                                               aspect=float(render_width) /
                                               float(render_height),
                                               nearVal=nearval,
                                               farVal=farval)
    (_, _, px, _, _) = p.getCameraImage(width=render_width,
                                        height=render_height,
                                        renderer=p.ER_BULLET_HARDWARE_OPENGL,
                                        viewMatrix=view_matrix,
                                        projectionMatrix=proj_matrix)
    rgb_array = np.array(px, dtype=np.uint8)
    rgb_array = np.reshape(np.array(px), (render_height, render_width, -1))
    rgb_array = rgb_array[:, :, :3]

    return rgb_array


def is_key_triggered(keys, key):
    o = ord(key)
    if o in keys:
        return keys[ord(key)] & p.KEY_WAS_TRIGGERED
    return False


def set_config(robot, joint_id, link_id, base_pos, base_quat, joint_pos):
    p.resetBasePositionAndOrientation(robot, base_pos, base_quat)
    for k, v in joint_pos.items():
        p.resetJointState(robot, joint_id[k], v, 0.)


def get_point_cloud_data(depth_buffer, view_matrix, projection_matrix, d_hor,
                         d_ver):
    view_matrix = np.asarray(view_matrix).reshape([4, 4], order='F')
    projection_matrix = np.asarray(projection_matrix).reshape([4, 4],
                                                              order='F')
    trans_world_to_pix = np.linalg.inv(
        np.matmul(projection_matrix, view_matrix))
    trans_camera_to_pix = np.linalg.inv(projection_matrix)
    img_height = (depth_buffer.shape)[0]
    img_width = (depth_buffer.shape)[1]

    wf_point_cloud_data = np.empty(
        [np.int(img_height / d_ver),
         np.int(img_width / d_hor), 3])
    cf_point_cloud_data = np.empty(
        [np.int(img_height / d_ver),
         np.int(img_width / d_hor), 3])

    for h in range(0, img_height, d_ver):
        for w in range(0, img_width, d_hor):
            x = (2 * w - img_width) / img_width
            y = (2 * h - img_height) / img_height
            z = 2 * depth_buffer[h, w] - 1
            pix_pos = np.asarray([x, y, z, 1])
            point_in_world = np.matmul(trans_world_to_pix, pix_pos)
            point_in_camera = np.matmul(trans_camera_to_pix, pix_pos)
            wf_point_cloud_data[np.int(h / d_ver),
                                np.int(w / d_hor), :] = (
                                    point_in_world /
                                    point_in_world[3])[:3]  #world frame

            cf_point_cloud_data[np.int(h / d_ver),
                                np.int(w / d_hor), :] = (
                                    point_in_world /
                                    point_in_world[3])[:3]  #camera frame

    return wf_point_cloud_data, cf_point_cloud_data
