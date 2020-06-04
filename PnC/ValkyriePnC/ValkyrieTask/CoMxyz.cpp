#include <Configuration.h>
#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieTask/CoMxyz.hpp>
#include <Utils/IO/IOUtilities.hpp>

CoMxyz::CoMxyz(RobotSystem* robot) : Task(robot, 3) {
  myUtils::pretty_constructor(3, "CoM xyz Task");
  Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
  JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
}

CoMxyz::~CoMxyz() {}

bool CoMxyz::_UpdateCommand(const Eigen::VectorXd& _pos_des,
                            const Eigen::VectorXd& _vel_des,
                            const Eigen::VectorXd& _acc_des) {
  // (X, Y, Z)
  for (int i = 0; i < 3; ++i) {
    pos_err[i] = _pos_des[i] - robot_->getCoMPosition()[i];
    vel_des[i] = _vel_des[i];
    acc_des[i] = _acc_des[i];
  }

  // com_vel_act
  Eigen::Vector3d com_vel_act = robot_->getCoMVelocity();

  // op_cmd
  for (int i(0); i < 3; ++i) {
    op_cmd[i] = acc_des[i] + kp_[i] * pos_err[i] +
                kd_[i] * (vel_des[i] - com_vel_act[i]);
  }

  // myUtils::pretty_print(des_ori, std::cout, "ori_des");
  // myUtils::pretty_print(ori_act, std::cout, "ori_act");
  // myUtils::pretty_print(pos_err, std::cout, "pos_err in
  // com_lin_body_ori_task");

  return true;
}

bool CoMxyz::_UpdateTaskJacobian() {
  // Eigen::MatrixXd Jtmp =
  // robot_->getBodyNodeCoMJacobian(ValkyrieBodyNode::pelvis);
  Eigen::MatrixXd Jtmp = robot_->getCoMJacobian();
  // (X, Y, Z)
  Jt_.block(0, 0, 3, robot_->getNumDofs()) =
      Jtmp.block(3, 0, 3, robot_->getNumDofs());

  // Replace Z Jacobian with pelvis.
  Jtmp = robot_->getBodyNodeCoMJacobian(ValkyrieBodyNode::pelvis);
  Jt_.block(2, 0, 1, robot_->getNumDofs()) =
      Jtmp.block(5, 0, 1, robot_->getNumDofs());

  return true;
}

bool CoMxyz::_UpdateTaskJDotQdot() {
  JtDotQdot_.setZero();
  return true;
}
