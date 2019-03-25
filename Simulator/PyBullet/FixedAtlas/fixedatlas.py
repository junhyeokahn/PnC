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
    print(j, " th joint")
    print(jointName)
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
    pos=np.zeros(activeJoint,dtype=float)
    vel=np.zeros(activeJoint,dtype=float)
    for i in range (activeJoint):
        joint_state =pb.getJointState(atlas,i)
        pos[i] = joint_state[0]
        vel[i] = joint_state[1]
    return pos, vel

fixedatlas_interface = FixedAtlasInterface.FixedAtlasInterface()
fixedatlas_sensordata = FixedAtlasInterface.FixedAtlasSensorData()
fixedatlas_command = FixedAtlasInterface.FixedAtlasCommand()

fixedatlas_sensordata.q = np.zeros(activeJoint)
fixedatlas_sensordata.qdot = np.zeros(activeJoint)
fixedatlas_command.jtrq = np.zeros(activeJoint)
fixedatlas_torque_command = np.zeros(activeJoint)

dt = 0.001
pb.setTimeStep(dt)
# pb.setPhysicsEngineParameter(fixedTimeStep=self.timestep*self.frame_skip, numSolverIterations=self.numSolverIterations, numSubSteps=self.frame_skip)

while (1):
    # pb.getCameraImage(320,200)
    # for i in range(len(paramIds)): ##JointControl from GUI
            # c=paramIds[i]
            # targetPos = pb.readUserDebugParameter(c)
            # pb.setJointMotorControl2(atlas,jointIds[i],pb.POSITION_CONTROL,targetPos,force=140.)
    pos, vel = GetJointStates()
    # print("==============pos=============")
    # print(pos)
    # print("================vel===========")
    # print(vel)

    fixedatlas_sensordata.q = pos
    fixedatlas_sensordata.qdot = vel

    fixedatlas_interface.getCommand(fixedatlas_sensordata,fixedatlas_command)

    fixedatlas_torque_command = fixedatlas_command.jtrq

    # print("===============torque============")
    # print(fixedatlas_torque_command)
    pb.setJointMotorControlArray(atlas,jointIds,controlMode=pb.TORQUE_CONTROL, forces=fixedatlas_torque_command)
    pb.stepSimulation()
    time.sleep(dt)
