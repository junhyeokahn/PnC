#include <Configuration.h>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/DracoTask/DCMTask.hpp>
#include <Utils/IO/IOUtilities.hpp>

DCMTask::DCMTask(RobotSystem* robot) : Task(robot, 3) {
  myUtils::pretty_constructor(3, "DCM Task");
  Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
  JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
  sp_ = DracoStateProvider::getStateProvider(robot_);
}

DCMTask::~DCMTask() {}

bool DCMTask::_UpdateCommand(const Eigen::VectorXd& _pos_des,
                             const Eigen::VectorXd& _vel_des,
                             const Eigen::VectorXd& _acc_des) {
  Eigen::VectorXd r_ic = sp_->dcm.head(2);
  Eigen::VectorXd r_ic_des = _pos_des.head(2);
  Eigen::VectorXd rdot_ic_des = _vel_des.head(2);
  double omega_o = std::sqrt(9.81 / _pos_des[2]);
  // double omega_o = std::sqrt(9.81 / robot_->getCoMPosition()[2]);
  Eigen::VectorXd r_cmp_des = Eigen::VectorXd::Zero(2);
  for (int i = 0; i < 2; ++i) {
    r_cmp_des[i] =
        r_ic[i] - rdot_ic_des[i] / omega_o + kp_[i] * (r_ic[i] - r_ic_des[i]);
  }

  op_cmd.head(2) = (9.81 / _pos_des[2]) * (sp_->com_pos.head(2) - r_cmp_des);
  op_cmd[2] = kp_[2] * (_pos_des[2] - sp_->com_pos[2]) +
              kd_[2] * (_vel_des[2] - sp_->est_com_vel[2]);

  return true;
}

bool DCMTask::_UpdateTaskJacobian() {
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

bool DCMTask::_UpdateTaskJDotQdot() {
  JtDotQdot_.setZero();
  return true;
}
