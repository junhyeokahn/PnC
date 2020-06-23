#include <Configuration.h>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoTask/Footxyz.hpp>
#include <Utils/IO/IOUtilities.hpp>

Footxyz::Footxyz(RobotSystem* robot, int _link_idx) : Task(robot, 3) {
  myUtils::pretty_constructor(3, "Foot xyz Task");

  Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
  JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
  link_idx_ = _link_idx;
}

Footxyz::~Footxyz() {}

bool Footxyz::_UpdateCommand(const Eigen::VectorXd& _pos_des,
                             const Eigen::VectorXd& _vel_des,
                             const Eigen::VectorXd& _acc_des) {
  // (X, Y, Z)
  for (int i = 0; i < 3; ++i) {
    // pos_err[i] = myUtils::bind_half_pi(ori_err_so3[i]);
    pos_err[i] =
        _pos_des[i] - robot_->getBodyNodeIsometry(link_idx_).translation()[i];
    vel_des[i] = _vel_des[i];
    acc_des[i] = _acc_des[i];
  }

  Eigen::VectorXd vel_act =
      robot_->getBodyNodeSpatialVelocity(link_idx_).tail(3);

  for (int i = 0; i < 3; ++i) {
    op_cmd[i] =
        acc_des[i] + kp_[i] * pos_err[i] + kd_[i] * (vel_des[i] - vel_act[i]);
  }

  // myUtils::pretty_print(des_ori, std::cout, "ori_des");
  // myUtils::pretty_print(ori_act, std::cout, "ori_act");
  // myUtils::pretty_print(pos_err, std::cout, "pos_err in bodyrpz");

  return true;
}

bool Footxyz::_UpdateTaskJacobian() {
  Eigen::MatrixXd Jtmp = robot_->getBodyNodeJacobian(link_idx_);
  Jt_.block(0, 0, 3, robot_->getNumDofs()) =
      Jtmp.block(3, 0, 3, robot_->getNumDofs());
  // isolate virtual joint
  Jt_.block(0, 0, dim_task_, robot_->getNumVirtualDofs()) =
      Eigen::MatrixXd::Zero(dim_task_, robot_->getNumVirtualDofs());

  return true;
}

bool Footxyz::_UpdateTaskJDotQdot() {
  Eigen::VectorXd v_tmp =
      robot_->getBodyNodeJacobianDot(link_idx_) * robot_->getQdot();
  JtDotQdot_ = v_tmp.segment(1, 2);

  JtDotQdot_.setZero();
  return true;
}
