import pybullet as pb
import pybullet_data
import time
import numpy as np
import yaml
import os
import sys
import ipdb 
PROJECT_PATH = os.getcwd()

sys.path.append(PROJECT_PATH + '/build/lib')
import FixedAtlasInterface

# =============================================================================
# Initial Configuration Setting
# =============================================================================
init_config=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

# =============================================================================
# Connect to Pysics Simulator From Client
# =============================================================================
pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.loadURDF("plane.urdf")
pb.setGravity(0,0,-9.81)
atlas = pb.loadURDF(PROJECT_PATH + \
        "/RobotModel/Robot/Atlas/FixedAtlasSim_PyBullet.urdf",[0,0,1], useFixedBase=True)

# =============================================================================
# Joint ID 
# =============================================================================
pb.setPhysicsEngineParameter(numSolverIterations=100)
pb.changeDynamics(atlas,-1,linearDamping=0,angularDamping=0)

jointIds=[]
activeJoint=0


for j in range (pb.getNumJoints(atlas)):
    pb.changeDynamics(atlas,j,linearDamping=0,angularDamping=0)
    info = pb.getJointInfo(atlas,j)
    jointName = info[1]
    jointType = info[2]
    linkName = info[12]
    print(j, " th joint")
    print(jointName)
    print(linkName)
    if (jointType==pb.JOINT_PRISMATIC or jointType==pb.JOINT_REVOLUTE):
            pb.setJointMotorControl2(atlas,j,controlMode=pb.VELOCITY_CONTROL,force=0.0) 
            jointIds.append(j)
            # print("this joint is active")
            pb.resetJointState(atlas,j,init_config[activeJoint]) #Set initial configuration for active joint
            activeJoint+=1

def so3_to_euler_zyx_dot(ori_ypr,angv_so3):
    x = ori_ypr[0]
    y = ori_ypr[1]
    z = ori_ypr[2]
    so3_to_euler_zyx_dot_map =np.array([[np.cos(z)*np.sin(y)/np.cos(y), np.sin(y)*np.sin(z)/np.cos(y),1],
                                        [-np.sin(z),np.cos(z),0],
                                        [np.cos(z)/np.cos(y), np.sin(z)/np.cos(y), 0]])
    angv_so3 = np.array(angv_so3)
    angv_euler = so3_to_euler_zyx_dot_map.dot(angv_so3)
    return angv_euler

def GetBaseStates():
    [base_pos, base_ori]=pb.getBasePositionAndOrientation(atlas)
    [base_vel, base_angv]=pb.getBaseVelocity(atlas) #angular velocity in so3
    base_ori_ypr = pb.getEulerFromQuaternion(base_ori)
    base_angv_euler = so3_to_euler_zyx_dot(base_ori_ypr,base_angv) #function mapping so3 to euler zyx vel
    base_pos_tot = list(base_pos) + list(base_ori_ypr)
    base_vel_tot = list(base_vel) + list(base_angv_euler) 
    return base_pos_tot, based_vel_tot

def GetJointStates():
    # pos = np.array([pb.getJointState(atlas, jidx)[0] for jidx in jointIds])
    # vel = np.array([pb.getJointState(atlas, jidx)[1] for jidx in jointIds])
    pos = [pb.getJointState(atlas, jidx)[0] for jidx in jointIds]
    vel = [pb.getJointState(atlas, jidx)[1] for jidx in jointIds]
    return pos, vel

fixedatlas_interface = FixedAtlasInterface.FixedAtlasInterface()
fixedatlas_sensordata = FixedAtlasInterface.FixedAtlasSensorData()
fixedatlas_command = FixedAtlasInterface.FixedAtlasCommand()

fixedatlas_sensordata.q = np.zeros(activeJoint)
fixedatlas_sensordata.qdot = np.zeros(activeJoint)
fixedatlas_command.jtrq = np.zeros(activeJoint)
fixedatlas_torque_command = np.zeros(activeJoint)
# =============================================================================
# Camera Setting
# =============================================================================
fov, aspect, nearplane, farplane = 90, 2.0, 0.01, 1000
projection_matrix = pb.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)
def atlas_camera():
    linkinfo = pb.getLinkState(atlas,10) #Get head link info
    com_pos = linkinfo[0]
    com_ori = linkinfo[1]
    rot_matrix = pb.getMatrixFromQuaternion(com_ori)
    rot_matrix = np.array(rot_matrix).reshape(3,3)
    #initial vectors
    global_camera_x = np.array([1,0,0])
    global_camera_z = np.array([0,0,1]) 
    #Rotated vectors
    init_camera_offset = rot_matrix.dot(0.1*global_camera_x)
    target_camera_offset = rot_matrix.dot(1.*global_camera_x)
    up_vector = rot_matrix.dot(global_camera_z)
    view_matrix = pb.computeViewMatrix(com_pos+init_camera_offset, com_pos +target_camera_offset, up_vector)
    img = pb.getCameraImage(320,200,view_matrix,projection_matrix)
    # rgb_matrix = img[2]
    # depth_matrix = img[3]
    # print("=============RGB=============")
    # print(rgb_matrix)
    # print("=============Depth=============")
    # print(depth_matrix)
    return img

# =============================================================================
# Simulation Setting
# =============================================================================
dt = 0.001
pb.setTimeStep(dt)
count = 0

while (1):
    if count % 33. == 0:
        img = atlas_camera()
    pos, vel = GetJointStates()

    fixedatlas_sensordata.q = np.array(pos)
    fixedatlas_sensordata.qdot = np.array(vel)

    fixedatlas_interface.getCommand(fixedatlas_sensordata,fixedatlas_command)

    fixedatlas_torque_command = fixedatlas_command.jtrq

    pb.setJointMotorControlArray(atlas,jointIds,controlMode=pb.TORQUE_CONTROL, forces=fixedatlas_torque_command)
    pb.stepSimulation()
    time.sleep(dt)
    count += 1
