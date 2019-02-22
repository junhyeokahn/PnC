import pybullet_data
import pybullet as pb
import time
import os
import yaml
import numpy as np
import ipdb
PROJECT_PATH = os.getcwd()

# =============================================================================
# Initial Configuration Setting
# =============================================================================
cfg_path = PROJECT_PATH + '/Config/Draco/SIMULATION.yaml'
with open(cfg_path) as f:
    config = yaml.safe_load(f)
    init_config = config['initial_configuration']
    hanging_height = config['hanging_height']

# =============================================================================
# Connect to Phsyics Client
# =============================================================================
pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.loadURDF("plane.urdf")
draco = pb.loadURDF(PROJECT_PATH + \
        "/RobotModel/Robot/Draco/DracoFixed.urdf", [0, 0, 0.5+hanging_height], \
        useFixedBase=False)


jointIds=[]
activeJoint=0

pb.setPhysicsEngineParameter(numSolverIterations=100)
pb.changeDynamics(draco,-1,linearDamping=0, angularDamping=0)

for j in range (pb.getNumJoints(draco)):
    info = pb.getJointInfo(draco,j)
    jointName = info[1]
    jointType = info[2]
    # print("------------")
    # print(j, " th joint")
    # print(jointName)
    if (jointType==pb.JOINT_PRISMATIC or jointType==pb.JOINT_REVOLUTE):
        # print("This is Active")
        jointIds.append(j)
        pb.resetJointState(draco, j, init_config[activeJoint])
        activeJoint+=1

def so3_to_euler_zyx_dot(so3,ypr):
    x = ypr[2]
    y = ypr[1]
    z = ypr[0]
    so3_to_euler_zyx_dot_map = \
            np.array([[np.cos(z)*np.sin(y)/np.cos(y), np.sin(y)*np.sin(z)/np.cos(y), 1],
                     [-np.sin(z), np.cos(z), 0],
                     [np.cos(z)/np.cos(y), np.sin(z)/np.cos(y), 0]])
    euler_zyx_dot_np = \
            (np.matmul(so3_to_euler_zyx_dot_map, np.array(so3).reshape(3, 1)))
    return np.squeeze(euler_zyx_dot_np).tolist()

def GetJointStates():
    [base_pos, base_quat] = pb.getBasePositionAndOrientation(draco)
    base_ypr = pb.getEulerFromQuaternion(base_quat)
    [base_vel, base_so3] = pb.getBaseVelocity(draco)
    euler_zyx_dot = so3_to_euler_zyx_dot(base_so3,base_ypr)

    pos = np.zeros(shape=(6+activeJoint), dtype=float)
    vel = np.zeros(shape=(6+activeJoint), dtype=float)
    for i, active_joint_idx in enumerate(jointIds):
        j_state = pb.getJointState(draco, active_joint_idx)
        pos[i] = j_state[0]
        vel[i] = j_state[1]

    pos = list(base_pos) + list(base_ypr) + list(pos)
    vel = list(base_vel) + euler_zyx_dot + list(vel)
    return pos, vel

pb.setRealTimeSimulation(1)
pb.setTimeStep(0.001)
pb.setGravity(0,0,-9.81)
while(1):
    pb.getCameraImage(320,200)
    # pb.setJointMotorControl2(draco,jointIds[0],pb.POSITION_CONTROL, t, force=140.)
    pos, vel = GetJointStates()
