import os
import inspect
import pybullet_envs
import gym
import argparse
import pybullet as p
import pybullet_envs
import pybullet_data
import time

def test(args):

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0, -9.81)
    p.loadURDF("plane.urdf")
    p.setTimeStep(1./240.)
    p.setRealTimeSimulation(0)

    if args.file.split('.')[-1] == 'urdf':
        b_urdf = True
    else:
        b_urdf = False

    if b_urdf:
        robot= p.loadURDF(args.file, [0, 0, 1.5])
    else:
        robot=p.loadMJCF(args.file)[0]

    joint_list = {}
    link_list = {}
    dof_idx = []
    active_joint = 0
    for j in range (p.getNumJoints(robot)):
        info = p.getJointInfo(robot, j)
        joint_name = info[1]
        joint_type = info[2]
        link_name = info[12]
        joint_list[joint_name.decode()] = j
        link_list[link_name.decode()] = j
        if (joint_type==p.JOINT_PRISMATIC or joint_type==p.JOINT_REVOLUTE):
            dof_idx.append(j)
            active_joint+=1
    n_dof = active_joint

    while (1):
        force= [0]*n_dof
        p.setJointMotorControlArray(robot, dof_idx, p.TORQUE_CONTROL, forces=force)
        p.stepSimulation()

def main():
    import argparse
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--file', help='environment ID', type=str)

    args = parser.parse_args()
    test(args)

if __name__ == '__main__':
    main()
