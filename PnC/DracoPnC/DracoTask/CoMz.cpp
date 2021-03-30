#include <Configuration.h>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/DracoTask/CoMz.hpp>
#include <Utils/IO/IOUtilities.hpp>

CoMz::CoMz(RobotSystem* robot) : Task(robot, 1) {
  myUtils::pretty_constructor(3, "CoM z Task");
  Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
  JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
  sp_ = DracoStateProvider::getStateProvider(robot_);
}

CoMz::~CoMz() {}

bool CoMz::_UpdateCommand(const Eigen::VectorXd& _pos_des,
                          const Eigen::VectorXd& _vel_des,
                          const Eigen::VectorXd& _acc_des) {
  Eigen::Vector3d com_pos = robot_->getCoMPosition();
  op_cmd[0] = acc_des[2] + kp_[0] * (_pos_des[2] - com_pos[2]) +
              kd_[0] * (vel_des[2] - sp_->est_com_vel[2]);
  return true;
}

bool CoMz::_UpdateCurrent(){
  // pos_cur
  pos_cur_ = robot_->getCoMPosition();
  // vel_cur
  vel_cur_ = robot_->getCoMVelocity();
 
  return true;
}

bool CoMz::_UpdateTaskJacobian() {
  // Replace Z Jacobian with pelvis.
  Eigen::MatrixXd Jtmp = robot_->getBodyNodeCoMJacobian(DracoBodyNode::Torso);
  Jt_ = Jtmp.block(5, 0, 1, robot_->getNumDofs());
  return true;
}

bool CoMz::_UpdateTaskJDotQdot() {
  JtDotQdot_ = (robot_->getCoMJacobianDot() * robot_->getQdot()).segment(2, 1);
  // JtDotQdot_.setZero();
  return true;
}
