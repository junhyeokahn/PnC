#include <Configuration.h>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoTask/TorsoRxRy.hpp>
#include <Utils/IO/IOUtilities.hpp>

TorsoRxRy::TorsoRxRy(RobotSystem* robot, int _link_idx) : Task(robot, 2) {
  myUtils::pretty_constructor(3, "Torso Rx Ry Task");

  Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
  JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
  link_idx_ = _link_idx;
}

TorsoRxRy::~TorsoRxRy() {}

bool TorsoRxRy::_UpdateCommand(const Eigen::VectorXd& _pos_des,
                               const Eigen::VectorXd& _vel_des,
                               const Eigen::VectorXd& _acc_des) {
  Eigen::Quaternion<double> des_ori(_pos_des[0], _pos_des[1], _pos_des[2],
                                    _pos_des[3]);
  Eigen::Quaternion<double> ori_act(
      robot_->getBodyNodeIsometry(link_idx_).linear());
  Eigen::Quaternion<double> quat_ori_err = des_ori * ori_act.inverse();
  Eigen::Vector3d ori_err_so3 = dart::math::quatToExp(quat_ori_err);

  // (Rx, Ry)
  for (int i = 0; i < 2; ++i) {
    pos_err[i] = (ori_err_so3[i]);
    vel_des[i] = _vel_des[i];
    acc_des[i] = _acc_des[i];
  }

  Eigen::VectorXd vel_act =
      robot_->getBodyNodeSpatialVelocity(link_idx_).head(2);

  for (int i(0); i < 2; ++i) {
    op_cmd[i] =
        acc_des[i] + kp_[i] * pos_err[i] + kd_[i] * (vel_des[i] - vel_act[i]);
  }

  return true;
}

bool TorsoRxRy::_UpdateTaskJacobian() {
  Eigen::MatrixXd Jtmp = robot_->getBodyNodeJacobian(link_idx_);
  // (Rx, Ry)
  Jt_ = Jtmp.block(0, 0, 2, robot_->getNumDofs());
  return true;
}

bool TorsoRxRy::_UpdateTaskJDotQdot() {
  Eigen::VectorXd v_tmp =
      robot_->getBodyNodeJacobianDot(link_idx_) * robot_->getQdot();
  JtDotQdot_ = v_tmp.head(2);

  // JtDotQdot_.setZero();
  return true;
}
