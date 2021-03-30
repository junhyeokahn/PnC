#include <Configuration.h>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoTask/FootLocalRyRzTask.hpp>
#include <Utils/IO/IOUtilities.hpp>

FootLocalRyRzTask::FootLocalRyRzTask(RobotSystem* robot, int _link_idx)
    : Task(robot, 2) {
  myUtils::pretty_constructor(3, "Foot Local Ry Rz Task");

  Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
  JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
  link_idx_ = _link_idx;
}

FootLocalRyRzTask::~FootLocalRyRzTask() {}

bool FootLocalRyRzTask::_UpdateCommand(const Eigen::VectorXd& _pos_des,
                                       const Eigen::VectorXd& _vel_des,
                                       const Eigen::VectorXd& _acc_des) {
  Eigen::Quaternion<double> des_ori(_pos_des[0], _pos_des[1], _pos_des[2],
                                    _pos_des[3]);
  Eigen::Quaternion<double> ori_act(
      robot_->getBodyNodeIsometry(link_idx_).linear());
  Eigen::Quaternion<double> quat_ori_err = des_ori * ori_act.inverse();
  Eigen::Vector3d ori_err_so3 = dart::math::quatToExp(quat_ori_err);

  // (Ry, Rz)
  for (int i = 0; i < 2; ++i) {
    // pos_err[i] = myUtils::bind_half_pi(ori_err_so3[i]);
    pos_err[i] = (ori_err_so3[i + 1]);
    vel_des[i] = _vel_des[i + 1];
    acc_des[i] = _acc_des[i + 1];
  }

  Eigen::VectorXd vel_act =
      robot_->getBodyNodeSpatialVelocity(link_idx_).head(3).tail(2);

  for (int i(0); i < 2; ++i) {
    op_cmd[i] =
        acc_des[i] + kp_[i] * pos_err[i] + kd_[i] * (vel_des[i] - vel_act[i]);
  }

  return true;
}

bool FootLocalRyRzTask::_UpdateCurrent() {
  Eigen::Quaternion<double> ori_zero(0.0, 0.0, 0.0, 1.0);
  Eigen::Quaternion<double> ori_act(
      robot_->getBodyNodeCoMIsometry(link_idx_).linear());

  myUtils::avoid_quat_jump(ori_zero, ori_act);

  Eigen::Vector3d ori_cur;
  ori_cur = dart::math::quatToExp(ori_act);
  pos_cur_ = ori_cur;

  // vel_cur_
  vel_cur_ = robot_->getBodyNodeCoMSpatialVelocity(link_idx_).head(3);

  return true;
}

bool FootLocalRyRzTask::_UpdateTaskJacobian() {
  Eigen::MatrixXd Jtmp = robot_->getBodyNodeJacobian(link_idx_);
  // (Ry, Rz)
  Jt_.block(0, 0, 2, robot_->getNumDofs()) =
      Jtmp.block(1, 0, 2, robot_->getNumDofs());
  // isolate virtual joint
  Jt_.block(0, 0, dim_task_, robot_->getNumVirtualDofs()) =
      Eigen::MatrixXd::Zero(dim_task_, robot_->getNumVirtualDofs());

  return true;
}

bool FootLocalRyRzTask::_UpdateTaskJDotQdot() {
  Eigen::VectorXd v_tmp =
      robot_->getBodyNodeJacobianDot(link_idx_) * robot_->getQdot();
  JtDotQdot_ = v_tmp.segment(1, 2);

  // JtDotQdot_.setZero();
  return true;
}
