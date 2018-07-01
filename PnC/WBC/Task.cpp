#include "Task.hpp"
#include "RobotSystem.hpp"
#include "DataManager.hpp"
#include "Utilities.hpp"
#include <dart/dart.hpp>

Task::Task(RobotSystem* robot_, TaskType taskType_, std::string linkName_) {
    mRobot = robot_;
    mIsUpdated = false;
    mTaskType = taskType_;
    mLinkName = linkName_;
    switch (taskType_) {
        case TaskType::JOINT:
            mDim = mRobot->getNumActuatedDofs(); // 34
            mType = "Joint";
            break;
        case TaskType::LINKXYZ:
            mDim = 3;
            mType = "LinkXYZ";
            break;
        case TaskType::LINKRPY:
            mDim = 3;
            mType = "LinkRPY";
            break;
        case TaskType::COM:
            mDim = 3;
            mType = "CoM";
            break;
        default:
            std::cout << "[Task] Type is not Specified" << std::endl;
    }
    mTaskCmd = Eigen::VectorXd::Zero(mDim);
    mJt = Eigen::MatrixXd::Zero(mDim, mRobot->getNumDofs());
    mJtDotQDot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    mKp = Eigen::VectorXd::Zero(mDim);
    mKd = Eigen::VectorXd::Zero(mDim);

    // Variables for DataManager
    mPosDes = Eigen::VectorXd::Zero(mDim);
    mVelDes = Eigen::VectorXd::Zero(mDim);
    mPosAct = Eigen::VectorXd::Zero(mDim);
    mVelAct = Eigen::VectorXd::Zero(mDim);

    DataManager* dataManager = DataManager::GetDataManager();
    dataManager->RegisterData(&mPosDes, VECT, mType+"TaskPosDes", mDim);
    dataManager->RegisterData(&mVelDes, VECT, mType+"TaskVelDes", mDim);
    dataManager->RegisterData(&mPosAct, VECT, mType+"TaskPosAct", mDim);
    dataManager->RegisterData(&mVelAct, VECT, mType+"TaskVelAct", mDim);

    printf("[Task %s] is Constructed\n", mType.c_str());
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
        Eigen::Quaternion<double> ori_cmd(pos_des_[0], pos_des_[1], pos_des_[2], pos_des_[3]);

        // Set actual value & gains
        Eigen::Quaternion<double> ori_act;
        Eigen::VectorXd vel_act= Eigen::VectorXd::Zero(mDim);
        ori_act = mRobot->getBodyNodeCoMIsometry(mLinkName).linear();
        vel_act = mRobot->getBodyNodeCoMSpatialVelocity(mLinkName).head(3);
        for (int i = 0; i < mDim; ++i) {
            mKp[i] = 150.;
            mKd[i] = 10.;
        }

        // Compute task command
        Eigen::Quaternion<double> quat_ori_err;
        Eigen::Vector3d ori_err;
        quat_ori_err = ori_cmd * (ori_act.inverse());
        ori_err = dart::math::quatToExp(quat_ori_err);
        for (int i = 0; i < 3; ++i)
            ori_err[i] = myUtils::bind_half_pi(ori_err[i]);

        for(int i(0); i < mDim; ++i){
            mTaskCmd[i] = acc_des_[i]
                + mKp[i] * ori_err[i]
                + mKd[i] * (vel_des_[i] - vel_act[i]);
        }

    } else {
        // Set actual value & gains
        Eigen::VectorXd pos_act = Eigen::VectorXd::Zero(mDim);
        Eigen::VectorXd vel_act= Eigen::VectorXd::Zero(mDim);
        switch (mTaskType) {
            case TaskType::JOINT:{
                                     pos_act = mRobot->getQ().tail(mDim);
                                     vel_act = mRobot->getQdot().tail(mDim);
                                     for (int i = 0; i < mDim; ++i) {
                                         mKp[i] = 300.;
                                         mKd[i] = 5.;
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
            case TaskType::COM:{
                                        pos_act = mRobot->getCoMPosition();
                                        vel_act = mRobot->getCentroidVelocity();
                                        for (int i = 0; i < mDim; ++i) {
                                            mKp[i] = 100.;
                                            mKd[i] = 5.;
                                        }
                                        break;
                                    }
            default:
                                    std::cout << "[Task] Type is not Specified" << std::endl;
        }

        // Compute task command
        for (int i = 0; i < mDim; ++i) {
            mTaskCmd[i] = acc_des_[i] +
                mKp[i] * (pos_des_[i] - pos_act[i]) +
                mKd[i] * (vel_des_[i] - vel_act[i]);
        }
        _saveTask(pos_des_, vel_des_, pos_act, vel_act);
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
        case TaskType::COM:{
                                    mJt = (mRobot->getCentroidJacobian()).block(
                                            3, 0, mDim, mRobot->getNumDofs());
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
        case TaskType::COM:{
                                    mJtDotQDot = (mRobot->getCentroidJacobian() *
                                        mRobot->getInvMassMatrix() *
                                        mRobot->getCoriolis()).tail(mDim);
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
                     const Eigen::VectorXd & pos_,
                     const Eigen::VectorXd & vel_) {
    mPosDes = pos_des_;
    mVelDes = vel_des_;
    mPosAct = pos_;
    mVelAct = vel_;
}
