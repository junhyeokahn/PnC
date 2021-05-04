#include <assert.h>

#include <Configuration.h>
#include <PnC/WBC/BasicTask.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

BasicTask::BasicTask(RobotSystem* _robot, const BasicTaskType& _taskType,
                     const int& _dim, const int& _link_idx)
    : Task(_robot, _dim) {
  task_type_ = _taskType;
  link_idx_ = _link_idx;
  switch (task_type_) {
    case BasicTaskType::JOINT:
      assert(dim_task_ = robot_->getNumActuatedDofs());
      task_type_string_ = "Joint";
      for (int i = 0; i < dim_task_; ++i) {
        kp_[i] = 0.;
        kd_[i] = 0.;
      }
      break;
    case BasicTaskType::LINKXYZ:
      assert(dim_task_ = 3);
      task_type_string_ = "LinkXYZ";
      for (int i = 0; i < dim_task_; ++i) {
        kp_[i] = 0.;
        kd_[i] = 0.;
      }
      break;
    case BasicTaskType::LINKORI:
      assert(dim_task_ = 3);
      task_type_string_ = "LinkRPY";
      for (int i = 0; i < dim_task_; ++i) {
        kp_[i] = 0.;
        kd_[i] = 0.;
      }
      break;
    case BasicTaskType::ISOLATED_LINKXYZ:
      assert(dim_task_ = 3);
      task_type_string_ = "Isolated LinkXYZ";
      for (int i = 0; i < dim_task_; ++i) {
        kp_[i] = 0.;
        kd_[i] = 0.;
      }
      break;
    case BasicTaskType::ISOLATED_LINKORI:
      assert(dim_task_ = 3);
      task_type_string_ = "Isolated LinkRPY";
      for (int i = 0; i < dim_task_; ++i) {
        kp_[i] = 0.;
        kd_[i] = 0.;
      }
      break;
    case BasicTaskType::CENTROID:
      assert(dim_task_ = 6);
      task_type_string_ = "Centroid";
      for (int i = 0; i < 3; ++i) {
        kp_[i] = 0.;
        kd_[i] = 0.;
      }
      for (int i = 3; i < 6; ++i) {
        kp_[i] = 0.;
        kd_[i] = 0.;
      }
      break;
    case BasicTaskType::COM:
      assert(dim_task_ = 3);
      task_type_string_ = "CoM";
      for (int i = 0; i < 3; ++i) {
        kp_[i] = 0.;
        kd_[i] = 0.;
      }
      break;
    default:
      std::cout << "[BasicTask] Type is not Specified" << std::endl;
  }
  myUtils::pretty_constructor(3, "Basic Task " + task_type_string_);
}

bool BasicTask::_UpdateCommand(const Eigen::VectorXd& _pos_des,
                               const Eigen::VectorXd& _vel_des,
                               const Eigen::VectorXd& _acc_des) {
  // vel_des, acc_des
  vel_des = _vel_des;
  acc_des = _acc_des;
  Eigen::VectorXd vel_act = Eigen::VectorXd::Zero(dim_task_);

  switch (task_type_) {
    case BasicTaskType::LINKORI: {
      // pos_err
      Eigen::Quaternion<double> ori_des(_pos_des[0], _pos_des[1], _pos_des[2],
                                        _pos_des[3]);
      Eigen::Quaternion<double> ori_act(
          robot_->getBodyNodeCoMIsometry(link_idx_).linear());

      // std::cout << "======================================" << std::endl;
      // myUtils::pretty_print(_pos_des, std::cout, "pos_des");
      // myUtils::pretty_print(ori_des, std::cout, "ori_des before");
      // myUtils::pretty_print(ori_act, std::cout, "ori_act before");
      myUtils::avoid_quat_jump(ori_des, ori_act);
      // myUtils::pretty_print(ori_des, std::cout, "ori_des after");
      // myUtils::pretty_print(ori_act, std::cout, "ori_act after");

      Eigen::Quaternion<double> quat_ori_err;
      quat_ori_err = ori_des * (ori_act.inverse());
      // myUtils::pretty_print(quat_ori_err, std::cout, "quat_ori_err");
      Eigen::Vector3d ori_err;

      ori_err = dart::math::quatToExp(quat_ori_err);
      // myUtils::pretty_print(ori_err, std::cout, "ori_err");
      for (int i = 0; i < 3; ++i) {
        // ori_err[i] = myUtils::bind_half_pi(ori_err[i]);
      }
      pos_err = ori_err;

      // vel_act
      vel_act = robot_->getBodyNodeCoMSpatialVelocity(link_idx_).head(3);
      // myUtils::pretty_print(pos_err, std::cout, "pos_err in ori");
      break;
    }
    case BasicTaskType::ISOLATED_LINKORI: {
      // pos_err
      Eigen::Quaternion<double> ori_des(_pos_des[0], _pos_des[1], _pos_des[2],
                                        _pos_des[3]);
      Eigen::Quaternion<double> ori_act(
          robot_->getBodyNodeCoMIsometry(link_idx_).linear());

      myUtils::avoid_quat_jump(ori_des, ori_act);

      Eigen::Quaternion<double> quat_ori_err;
      quat_ori_err = ori_des * (ori_act.inverse());
      Eigen::Vector3d ori_err;
      ori_err = dart::math::quatToExp(quat_ori_err);
      for (int i = 0; i < 3; ++i) {
        // ori_err[i] = myUtils::bind_half_pi(ori_err[i]);
      }
      pos_err = ori_err;

      // vel_act
      vel_act = robot_->getBodyNodeCoMSpatialVelocity(link_idx_).head(3);
      // myUtils::pretty_print(pos_err, std::cout, "pos_err in ori");
      break;
    }
    case BasicTaskType::JOINT: {
      // pos_err
      pos_err = _pos_des - robot_->getQ().tail(dim_task_);
      // vel_act
      vel_act = robot_->getQdot().tail(dim_task_);
      break;
    }
    case BasicTaskType::LINKXYZ: {
      // pos_err
      pos_err =
          _pos_des - robot_->getBodyNodeCoMIsometry(link_idx_).translation();
      // vel_act
      vel_act = robot_->getBodyNodeCoMSpatialVelocity(link_idx_).tail(3);
      break;
    }
    case BasicTaskType::ISOLATED_LINKXYZ: {
      // pos_err
      pos_err =
          _pos_des - robot_->getBodyNodeCoMIsometry(link_idx_).translation();
      // vel_act
      vel_act = robot_->getBodyNodeCoMSpatialVelocity(link_idx_).tail(3);
      break;
    }
    case BasicTaskType::CENTROID: {
      // pos_err
      pos_err.head(3) = Eigen::VectorXd::Zero(3);
      pos_err.tail(3) = _pos_des.tail(3) - robot_->getCoMPosition();
      // vel_act
      vel_act = robot_->getCentroidMomentum();
      break;
    }
    case BasicTaskType::COM: {
      // pos_err
      pos_err = _pos_des - robot_->getCoMPosition();
      // vel_act
      vel_act = robot_->getCoMVelocity();
      break;
    }
    default:
      std::cout << "[BasicTask] Type is not Specified" << std::endl;
  }

  // op_cmd
  for (int i(0); i < dim_task_; ++i) {
    op_cmd[i] =
        acc_des[i] + kp_[i] * pos_err[i] + kd_[i] * (vel_des[i] - vel_act[i]);
  }

  return true;
}

bool BasicTask::_UpdateTaskJacobian() {
  switch (task_type_) {
    case BasicTaskType::JOINT: {
      (Jt_.block(0, robot_->getNumVirtualDofs(), dim_task_,
                 robot_->getNumActuatedDofs()))
          .setIdentity();
      break;
    }
    case BasicTaskType::LINKXYZ: {
      Jt_ = (robot_->getBodyNodeCoMJacobian(link_idx_))
                .block(3, 0, dim_task_, robot_->getNumDofs());
      break;
    }
    case BasicTaskType::ISOLATED_LINKXYZ: {
      Jt_ = (robot_->getBodyNodeCoMJacobian(link_idx_))
                .block(3, 0, dim_task_, robot_->getNumDofs());
      Jt_.block(0, 0, dim_task_, robot_->getNumVirtualDofs()) =
          Eigen::MatrixXd::Zero(dim_task_, robot_->getNumVirtualDofs());
      break;
    }
    case BasicTaskType::LINKORI: {
      Jt_ = (robot_->getBodyNodeCoMJacobian(link_idx_))
                .block(0, 0, dim_task_, robot_->getNumDofs());
      break;
    }
    case BasicTaskType::ISOLATED_LINKORI: {
      Jt_ = (robot_->getBodyNodeCoMJacobian(link_idx_))
                .block(0, 0, dim_task_, robot_->getNumDofs());
      Jt_.block(0, 0, dim_task_, robot_->getNumVirtualDofs()) =
          Eigen::MatrixXd::Zero(dim_task_, robot_->getNumVirtualDofs());
      break;
    }
    case BasicTaskType::CENTROID: {
      Jt_ = robot_->getCentroidInertiaTimesJacobian();
      break;
    }
    case BasicTaskType::COM: {
      Jt_ =
          robot_->getCoMJacobian().block(3, 0, dim_task_, robot_->getNumDofs());
      break;
    }
    default: { std::cout << "[BasicTask] Type is not Specified" << std::endl; }
  }
  return true;
}

bool BasicTask::_UpdateTaskJDotQdot() {
  switch (task_type_) {
    case BasicTaskType::JOINT: {
      JtDotQdot_.setZero();
      break;
    }
    case BasicTaskType::LINKXYZ: {
      JtDotQdot_ = robot_->getBodyNodeCoMJacobianDot(link_idx_).block(
                       3, 0, dim_task_, robot_->getNumDofs()) *
                   robot_->getQdot();
      break;
    }
    case BasicTaskType::LINKORI: {
      JtDotQdot_ = robot_->getBodyNodeCoMJacobianDot(link_idx_).block(
                       0, 0, dim_task_, robot_->getNumDofs()) *
                   robot_->getQdot();
      break;
    }
    case BasicTaskType::ISOLATED_LINKXYZ: {
      Eigen::MatrixXd Jdot = robot_->getBodyNodeCoMJacobianDot(link_idx_).block(
          3, 0, dim_task_, robot_->getNumDofs());
      Jdot.block(0, 0, dim_task_, robot_->getNumVirtualDofs()) =
          Eigen::MatrixXd::Zero(dim_task_, robot_->getNumVirtualDofs());
      JtDotQdot_ = Jdot * robot_->getQdot();
      break;
    }
    case BasicTaskType::ISOLATED_LINKORI: {
      Eigen::MatrixXd Jdot = robot_->getBodyNodeCoMJacobianDot(link_idx_).block(
          0, 0, dim_task_, robot_->getNumDofs());
      Jdot.block(0, 0, dim_task_, robot_->getNumVirtualDofs()) =
          Eigen::MatrixXd::Zero(dim_task_, robot_->getNumVirtualDofs());
      JtDotQdot_ = Jdot * robot_->getQdot();
      break;
    }
    case BasicTaskType::CENTROID: {
      // JtDotQdot_ = robot_->getCentroidJacobian() *
      // robot_->getInvMassMatrix() *
      // robot_->getCoriolis();
      JtDotQdot_.setZero();
      break;
    }
    case BasicTaskType::COM: {
      JtDotQdot_.setZero();
      break;
    }
    default: { std::cout << "[BasicTask] Type is not Specified" << std::endl; }
  }
  return true;
}
