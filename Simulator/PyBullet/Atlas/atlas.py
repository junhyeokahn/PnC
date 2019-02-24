import pybullet as pb
import pybullet_data
import time
import numpy as np
import yaml
import os
import ipdb 
PROJECT_PATH = os.getcwd()

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
atlas = pb.loadURDF(PROJECT_PATH + \
        "/RobotModel/Robot/Atlas/atlas_v4_with_multisense.urdf", [0,0,1], useFixedBase=True)

# =============================================================================
# Joint ID 
# =============================================================================
pb.setPhysicsEngineParameter(numSolverIterations=100)
pb.changeDynamics(atlas,-1,linearDamping=0,angularDamping=0)

gravId = pb.addUserDebugParameter("gravity",-10,10,-10) ##
paramIds=[] ##

jointIds=[]
activeJoint=0


for j in range (pb.getNumJoints(atlas)):
    pb.changeDynamics(atlas,j,linearDamping=0,angularDamping=0)
    info = pb.getJointInfo(atlas,j)
    jointName = info[1]
    jointType = info[2]
    # print(j, " th joint")
    # print(jointName)
    if (jointType==pb.JOINT_PRISMATIC or jointType==pb.JOINT_REVOLUTE):
            jointIds.append(j)
            # print("this joint is active")
            paramIds.append(pb.addUserDebugParameter(jointName.decode("utf-8"),-4,4,init_config[activeJoint]))
            pb.resetJointState(atlas,j,init_config[activeJoint]) #Set initial configuration for active joint
            activeJoint+=1

def so3_to_euler_zyx_dot(ori_ypr,angv_so3):
    x = ori_ypr[2]
    y = ori_ypr[1]
    z = ori_ypr[0]
    so3_to_euler_zyx_dot_map =np.array([[np.cos(z)*np.sin(y)/np.cos(y), np.sin(y)*np.sin(z)/np.cos(y),1],
                                        [-np.sin(z),np.cos(z),0],
                                        [np.cos(z)/np.cos(y), np.sin(z)/np.cos(y), 0]])
    angv_so3 = np.array(angv_so3)
    angv_euler = so3_to_euler_zyx_dot_map.dot(angv_so3)
    return angv_euler

def GetJointStates():
    [base_pos, base_ori]=pb.getBasePositionAndOrientation(atlas)
    [base_vel, base_angv]=pb.getBaseVelocity(atlas) #angular velocity in so3
    base_ori_ypr = pb.getEulerFromQuaternion(base_ori)
    base_angv_euler = so3_to_euler_zyx_dot(base_ori_ypr,base_angv) #function mapping so3 to euler zyx vel
    
    pos=np.zeros(activeJoint,dtype=float)
    vel=np.zeros(activeJoint,dtype=float)

    for i in range (activeJoint):
        joint_state =pb.getJointState(atlas,i)
        pos[i] = joint_state[0]
        vel[i] = joint_state[1]
    
    pos_tot = list(base_pos) + list(base_ori_ypr) + list(pos)
    vel_tot = list(base_vel) + list(base_angv_euler) + list(vel)
    return pos_tot, vel_tot


# pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING,0)
# for i in range (pb.getNumJoints(atlas)):
        # pb.setJointMotorControl2(atlas,i,pb.POSITION_CONTROL,0)
        # print(pb.getJointInfo(atlas,i))

# if 1:
        # objs = pb.loadSDF("botlab/botlab.sdf", globalScaling=2.0)
        # zero=[0,0,0]
        # pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING,1)
        # print("converting y to z axis")
        # for o in objs:
                # pos,orn = pb.getBasePositionAndOrientation(o)
                # y2x = pb.getQuaternionFromEuler([3.14/2.,0,3.14/2])
                # newpos,neworn = pb.multiplyTransforms(zero,y2x,pos,orn)
                # pb.resetBasePositionAndOrientation(o,newpos,neworn)
# else:
        # pb.loadURDF("plane.urdf",[0,0,-3])

# pb.loadURDF("boston_box.urdf",[-2,3,-2], useFixedBase=True)

# pb.resetDebugVisualizerCamera( cameraDistance=1, cameraYaw=148, cameraPitch=-9, cameraTargetPosition=[0.36,5.3,-0.62])

# pb.loadURDF("boston_box.urdf",[0,3,-2],useFixedBase=True)

# pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING,1)



# t=0
pb.setRealTimeSimulation(1)
pb.setTimeStep(0.001)
while (1):
    pb.getCameraImage(320,200)#, renderer=pb.ER_BULLET_HARDWARE_OPENGL )
    pb.setGravity(0,0, pb.readUserDebugParameter(gravId))
    for i in range(len(paramIds)): ##JointControl from GUI
            c=paramIds[i]
            targetPos = pb.readUserDebugParameter(c)
            pb.setJointMotorControl2(atlas,jointIds[i],pb.POSITION_CONTROL,targetPos,force=140.)
    pos_tot,vel_tot = GetJointStates()
        # time.sleep(0.01)
        # t+=0.01
        # keys = pb.getKeyboardEvents()
        # for k in keys:
                # if (keys[k]&pb.KEY_WAS_TRIGGERED):
                        # if (k == ord('i')):
                                # x = 10.*math.sin(t)
                                # y = 10.*math.cos(t)
                                # pb.getCameraImage(320,200,lightDirection=[x,y,10],shadow=1)#, renderer=pb.ER_BULLET_HARDWARE_OPENGL )


