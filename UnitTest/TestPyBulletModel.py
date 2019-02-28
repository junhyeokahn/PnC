import os
import inspect
import pybullet_envs
import gym
import argparse
import pybullet as p
import pybullet_envs
import pybullet_data
import time
import numpy as np

def test(args):
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    # p.setGravity(0, 0, 0)
    p.loadURDF("plane.urdf")
    p.setTimeStep(1./240.)

    if args.file.split('.')[-1] == 'urdf':
        b_urdf = True
    else:
        b_urdf = False

    if b_urdf:
        robot= p.loadURDF(args.file, [0, 0, 1.5], useFixedBase=True)
    else:
        robot=p.loadMJCF(args.file)[0]

    joint_list = {}
    link_list = {}
    dof_idx = []
    active_joint = 0
    for j in range (p.getNumJoints(robot)):
        # ======================================================================
        # Print Infos
        # ======================================================================
        info = p.getJointInfo(robot, j)
        print(j, " th joint")
        print("j damping : ", info[6])
        print("j fric : ", info[7])
        linfo = p.getDynamicsInfo(robot, j)
        print(j, " th link")
        print("mass : ", linfo[0])
        print("friction : ", linfo[1])
        print("local indertia: ", linfo[2])

        joint_name = info[1]
        joint_type = info[2]
        link_name = info[12]
        joint_list[joint_name.decode()] = j
        link_list[link_name.decode()] = j
        p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL,force=0.0)
        if (joint_type==p.JOINT_PRISMATIC or joint_type==p.JOINT_REVOLUTE):
            dof_idx.append(j)
            active_joint+=1
    n_dof = active_joint

    print(joint_list)
    print(dof_idx)

    # p.resetJointState(robot, joint_list['rAnkle'], np.pi/2.)
    # p.resetJointState(robot, joint_list['lAnkle'], np.pi/2.)

    while (1):
        p.stepSimulation()
        time.sleep(1./240.)

def main():
    import argparse
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--file', help='environment ID', type=str)

    args = parser.parse_args()
    test(args)

if __name__ == '__main__':
    main()
