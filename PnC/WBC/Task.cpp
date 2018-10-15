#include "Configuration.h"
#include "PnC/WBC/Task.hpp"
#include "RobotSystem/RobotSystem.hpp"
#include "Utils/DataManager.hpp"
#include "Utils/Utilities.hpp"
#include <dart/dart.hpp>

Task::Task(RobotSystem* robot_, TaskType taskType_, std::string linkName_) {
    mRobot = robot_;
    mIsUpdated = false;
    mTaskType = taskType_;
    mLinkName = linkName_;
    switch (taskType_) {
        case TaskType::JOINT:
            mDim = mRobot->getNumActuatedDofs(); // 34
            mTypeString = "Joint";
            break;
        case TaskType::LINKXYZ:
            mDim = 3;
            mTypeString = "LinkXYZ";
            break;
        case TaskType::LINKRPY:
            mDim = 3;
            mTypeString = "LinkRPY";
            break;
        case TaskType::CENTROID:
            mDim = 6;
            mTypeString = "Centroid";
            break;
        case TaskType::BASERPZ:
            mDim = 3;
            mTypeString = "BaseRPZ";
            break;
        case TaskType::COMRPY:
            mDim = 6;
            mTypeString = "CoMRPY";
            break;
        default:
            std::cout << "[Task] Type is not Specified" << std::endl;
    }
    mAccCmd = Eigen::VectorXd::Zero(mDim);
    mVelCmd = Eigen::VectorXd::Zero(mDim);
    mJt = Eigen::MatrixXd::Zero(mDim, mRobot->getNumDofs());
    mJtDotQDot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    mKp = Eigen::VectorXd::Zero(mDim);
    mKd = Eigen::VectorXd::Zero(mDim);
    mKi = Eigen::VectorXd::Zero(mDim);
    mErrSum = Eigen::VectorXd::Zero(mDim);

    // Variables for DataManager
    mPosDes = Eigen::VectorXd::Zero(mDim);
    mVelDes = Eigen::VectorXd::Zero(mDim);
    mPosAct = Eigen::VectorXd::Zero(mDim);
    mVelAct = Eigen::VectorXd::Zero(mDim);
    mAccDes = Eigen::VectorXd::Zero(mDim);
    mPosErr = Eigen::VectorXd::Zero(mDim);

    DataManager* dataManager = DataManager::GetDataManager();
    dataManager->RegisterData(&mPosDes, VECT, mTypeString+"TaskPosDes", mDim);
    dataManager->RegisterData(&mVelDes, VECT, mTypeString+"TaskVelDes", mDim);
    dataManager->RegisterData(&mPosAct, VECT, mTypeString+"TaskPosAct", mDim);
    dataManager->RegisterData(&mVelAct, VECT, mTypeString+"TaskVelAct", mDim);

    printf("[Task %s] is Constructed\n", mTypeString.c_str());
}

Task::~Task() {}

void Task::updateTaskSpec(const Eigen::VectorXd & pos_des_,
        const Eigen::VectorXd & vel_des_,
        const Eigen::VectorXd & acc_des_) {
    _updateCommand(pos_des_, vel_des_, acc_des_);
    _updateJt();
    _updateJtDotQDot();
    mIsUpdated = true;
}

void Task::_updateCommand(const Eigen::VectorXd & pos_des_,
        const Eigen::VectorXd & vel_des_,
        const Eigen::VectorXd & acc_des_) {
    if (mTaskType == TaskType::LINKRPY) {
        // Set desired value
        Eigen::Quaternion<double> ori_des(pos_des_[0], pos_des_[1], pos_des_[2], pos_des_[3]);

        // Set actual value & gains
        Eigen::Quaternion<double> ori_act;
        Eigen::VectorXd vel_act= Eigen::VectorXd::Zero(mDim);
        ori_act = mRobot->getBodyNodeCoMIsometry(mLinkName).linear();
        vel_act = mRobot->getBodyNodeCoMSpatialVelocity(mLinkName).head(3);
        for (int i = 0; i < mDim; ++i) {
            mKp[i] = 130.;
            mKd[i] = 10.;
        }

        // Compute task command
        Eigen::Quaternion<double> quat_ori_err;
        Eigen::Vector3d ori_err;
        quat_ori_err = ori_des * (ori_act.inverse());
        ori_err = dart::math::quatToExp(quat_ori_err);
        for (int i = 0; i < 3; ++i)
            ori_err[i] = myUtils::bind_half_pi(ori_err[i]);

        for(int i(0); i < mDim; ++i){
            mAccCmd[i] = acc_des_[i]
                + mKp[i] * ori_err[i]
                + mKd[i] * (vel_des_[i] - vel_act[i]);
        }

    } else if (mTaskType == TaskType::BASERPZ) {
        // Set desired value
        Eigen::Quaternion<double> ori_des(pos_des_[0], pos_des_[1], pos_des_[2], pos_des_[3]);
        double height_des(pos_des_[4]);

        // Set actual value
        Eigen::Quaternion<double> ori_act;
        ori_act = mRobot->getBodyNodeIsometry(mLinkName).linear();
        double height_act((mRobot->getQ())[5]);
        Eigen::VectorXd vel_act = Eigen::VectorXd::Zero(mDim);
        vel_act.head(2) = mRobot->getBodyNodeSpatialVelocity(mLinkName).head(2);
        vel_act[2] = mRobot->getBodyNodeSpatialVelocity(mLinkName)[5];

        for (int i = 0; i < mDim; ++i) {
            mKp[i] = 130.;
            mKd[i] = 10.;
        }

        // Compute task command
        Eigen::Quaternion<double> quat_ori_err;
        Eigen::Vector3d ori_err;
        quat_ori_err = ori_des * (ori_act.inverse());
        ori_err = dart::math::quatToExp(quat_ori_err);
        for (int i = 0; i < 3; ++i)
            ori_err[i] = myUtils::bind_half_pi(ori_err[i]);

        // Rx, Ry
        for(int i(0); i<2; ++i)
            mPosErr[i] = ori_err[i];
        mPosErr[2] = height_des - height_act;
        mVelDes = vel_des_; mAccDes = acc_des_;

        for(int i(0); i < mDim; ++i) {
            mAccCmd[i] = acc_des_[i]
                + mKp[i] * ori_err[i]
                + mKd[i] * (vel_des_[i] - vel_act[i]);
        }

    } else if (mTaskType == TaskType::COMRPY) {
        // Set desired value
        Eigen::Quaternion<double> ori_des(pos_des_[3], pos_des_[4], pos_des_[5], pos_des_[6]);
        Eigen::Vector3d pod_des = pos_des_.segment(0, 3);
        // Set actual value
        Eigen::Quaternion<double> ori_act;
        Eigen::VectorXd pos_act(3), vel_act(6);
        ori_act = mRobot->getBodyNodeIsometry(mLinkName).linear();
        pos_act = mRobot->getCoMPosition();
        vel_act.head(3) = mRobot->getCoMVelocity();
        vel_act.tail(3) = mRobot->getBodyNodeSpatialVelocity(mLinkName).head(3);
        // Compute orientation error
        Eigen::Quaternion<double> quat_ori_err;
        Eigen::Vector3d ori_err;
        quat_ori_err = ori_des * (ori_act.inverse());
        ori_err = dart::math::quatToExp(quat_ori_err);
        for (int i = 0; i < 3; ++i)
            ori_err[i] = myUtils::bind_half_pi(ori_err[i]);

        for (int i = 0; i < mDim; ++i) {
            mKp[i] = 10.;
            mKd[i] = 1.;
        }

        for (int i = 0; i < 3; ++i) {
            mAccCmd[i] = acc_des_[i] + mKp[i] * (pos_des_[i] - pos_act[i])
                + mKd[i] * (vel_des_[i] - vel_act[i]);
            mAccCmd[i+3] = acc_des_[i+3] + mKp[i+3] * ori_err[i]
                + mKd[i+3] *(vel_des_[i+3] - vel_act[i+3]);
        }
        std::cout << "virtual" << std::endl;
        std::cout << mRobot->getQ().segment(3, 3) << std::endl;
        std::cout << "ori err" << std::endl;
        std::cout << ori_err << std::endl;

    } else {
        // Set actual value & gains
        Eigen::VectorXd pos_act = Eigen::VectorXd::Zero(mDim);
        Eigen::VectorXd vel_act= Eigen::VectorXd::Zero(mDim);
        switch (mTaskType) {
            case TaskType::JOINT:{
                                     pos_act = mRobot->getQ().tail(mDim);
                                     vel_act = mRobot->getQdot().tail(mDim);
                                     for (int i = 0; i < mDim; ++i) {
                                         mKp[i] = 100.;
                                         mKd[i] = 5.;
                                         mKi[i] = 0.;
                                     }
                                     break;
                                 }
            case TaskType::LINKXYZ:{
                                       pos_act = mRobot->
                                           getBodyNodeCoMIsometry(mLinkName).translation();
                                       vel_act = mRobot->
                                           getBodyNodeCoMSpatialVelocity(mLinkName).tail(3);
                                       for (int i = 0; i < mDim; ++i) {
                                           mKp[i] = 150.;
                                           mKd[i] = 10.;
                                       }
                                       break;
                                   }
            case TaskType::CENTROID:{
                                        pos_act.head(3) = Eigen::VectorXd::Zero(3);
                                        pos_act.tail(3) = mRobot->getCoMPosition();
                                        vel_act = mRobot->getCentroidMomentum();
                                        for (int i = 0; i < 3; ++i) {
                                            mKp[i] = 0.;
                                            mKd[i] = 100.;
                                        }
                                        for (int i = 3; i < 6; ++i) {
                                            mKp[i] = 200.;
                                            mKd[i] = 20.;
                                        }
                                        break;
                                    }
            default:
                                    std::cout << "[Task] Type is not Specified" << std::endl;
        }

        // Compute task command
        for (int i = 0; i < mDim; ++i) {
            mAccCmd[i] = acc_des_[i] +
                mKp[i] * (pos_des_[i] - pos_act[i]) +
                mKd[i] * (vel_des_[i] - vel_act[i]);
            mVelCmd[i] = vel_des_[i] + mKp[i] * (pos_des_[i] - pos_act[i]);
        }
        _saveTask(pos_des_, vel_des_, acc_des_, pos_act, vel_act);
    }
}

void Task::_updateJt() {
    switch (mTaskType) {
        case TaskType::JOINT:{
                                 (mJt.block(0, mRobot->getNumVirtualDofs(),
                                  mDim, mRobot->getNumActuatedDofs())).setIdentity();
                                 break;
                             }
        case TaskType::LINKXYZ:{
                                   mJt = (mRobot->
                                           getBodyNodeCoMJacobian(mLinkName)).block(
                                           3, 0, mDim, mRobot->getNumDofs());
                                   break;
                               }
        case TaskType::LINKRPY:{
                                   mJt = (mRobot->
                                           getBodyNodeCoMJacobian(mLinkName)).block(
                                           0, 0, mDim, mRobot->getNumDofs());
                                   break;
                               }
        case TaskType::CENTROID:{
                                    mJt = mRobot->getCentroidInertiaTimesJacobian();
                                    break;
                                }
        case TaskType::BASERPZ:{
                                   Eigen::MatrixXd J_base =
                                       mRobot->getBodyNodeJacobian(mLinkName);
                                   mJt.block(0, 0, 2, mRobot->getNumDofs()) =
                                       J_base.block(0, 0, 2, mRobot->getNumDofs());
                                   mJt.block(0, 2, 1, mRobot->getNumDofs()) =
                                       J_base.block(0, 5, 1, mRobot->getNumDofs());
                                   break;
                               }
        case TaskType::COMRPY:{
                                  mJt.block(0, 0, 3, mRobot->getNumDofs()) =
                                      mRobot->getCoMJacobian().block(3, 0, 3, mRobot->getNumDofs());
                                  mJt.block(3, 3, 3, 3) =
                                      Eigen::MatrixXd::Identity(3, 3);
                                  break;
                              }
        default:{
                    std::cout << "[Task] Type is not Specified" << std::endl;
                }
    }
}

void Task::_updateJtDotQDot() {
    switch (mTaskType) {
        case TaskType::JOINT:{
                                 mJtDotQDot.setZero();
                                 break;
                             }
        case TaskType::LINKXYZ:{
                                   mJtDotQDot = mRobot->
                                       getBodyNodeCoMJacobianDot(mLinkName).block(3, 0, mDim, mRobot->getNumDofs());
                                   break;
                               }
        case TaskType::LINKRPY:{
                                   mJtDotQDot = mRobot->
                                       getBodyNodeCoMJacobianDot(mLinkName).block(0, 0, mDim, mRobot->getNumDofs());
                                   break;
                               }
        case TaskType::CENTROID:{
                                    //mJtDotQDot = mRobot->getCentroidJacobian() *
                                        //mRobot->getInvMassMatrix() *
                                        //mRobot->getCoriolis();
                                    mJtDotQDot.setZero(); // TODO : what is this
                                    break;
                                }
        case TaskType::BASERPZ:{
                                    mJtDotQDot.setZero();
                                    break;
                               }
        case TaskType::COMRPY:{
                                  mJtDotQDot.setZero();
                                  break;
                              }
        default:{
                    std::cout << "[Task] Type is not Specified" << std::endl;
                }
    }
}

void Task::setGain(const Eigen::VectorXd & kp_,
                    const Eigen::VectorXd & kd_) {
    mKp = kp_;
    mKd = kd_;
}

void Task::_saveTask(const Eigen::VectorXd & pos_des_,
                     const Eigen::VectorXd & vel_des_,
                     const Eigen::VectorXd & acc_des_,
                     const Eigen::VectorXd & pos_,
                     const Eigen::VectorXd & vel_) {
    mPosDes = pos_des_;
    mPosErr = pos_des_ - pos_;
    mVelDes = vel_des_;
    mPosAct = pos_;
    mVelAct = vel_;
    mAccDes = acc_des_;
}
