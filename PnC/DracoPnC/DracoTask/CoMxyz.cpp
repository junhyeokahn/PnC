#include <Configuration.h>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/DracoTask/CoMxyz.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <climits>
#include <cstdlib>

CoMxyz::CoMxyz(RobotSystem* robot) : Task(robot, 3) {
  myUtils::pretty_constructor(3, "CoM xyz Task");
  Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
  JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
  sp_ = DracoStateProvider::getStateProvider(robot_);
}

CoMxyz::~CoMxyz() {}

bool CoMxyz::_UpdateCommand(const Eigen::VectorXd& _pos_des,
                            const Eigen::VectorXd& _vel_des,
                            const Eigen::VectorXd& _acc_des) {
  Eigen::Vector3d com_pos = robot_->getCoMPosition();
  for (int i = 0; i < 3; ++i) {
    pos_err[i] = _pos_des[i] - com_pos[i];
    vel_des[i] = _vel_des[i];
    acc_des[i] = _acc_des[i];
  }

  for (int i(0); i < 3; ++i) {
    op_cmd[i] = acc_des[i] + kp_[i] * pos_err[i] +
                kd_[i] * (vel_des[i] - sp_->est_com_vel[i]);
  }

  return true;
}

bool CoMxyz::_UpdateCurrent(){
  // pos_cur
  pos_cur_ = robot_->getCoMPosition();
  // vel_cur
  vel_cur_ = robot_->getCoMVelocity();

  return true;
}

bool CoMxyz::_UpdateTaskJacobian() {
  Eigen::MatrixXd Jtmp = robot_->getCoMJacobian();
  // (X, Y, Z)
  Jt_.block(0, 0, 3, robot_->getNumDofs()) =
      Jtmp.block(3, 0, 3, robot_->getNumDofs());

  // Replace Z Jacobian with pelvis.
  Jtmp = robot_->getBodyNodeCoMJacobian(DracoBodyNode::Torso);
  Jt_.block(2, 0, 1, robot_->getNumDofs()) =
      Jtmp.block(5, 0, 1, robot_->getNumDofs());

  return true;
}

bool CoMxyz::_UpdateTaskJDotQdot() {
  JtDotQdot_ = robot_->getCoMJacobianDot() * robot_->getQdot();

  // JtDotQdot_.setZero();
  return true;
}
