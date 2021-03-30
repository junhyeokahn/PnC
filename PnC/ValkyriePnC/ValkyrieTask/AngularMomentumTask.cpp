#include <Configuration.h>
#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieTask/AngularMomentumTask.hpp>
#include <Utils/IO/IOUtilities.hpp>

AngularMomentumTask::AngularMomentumTask(RobotSystem* robot,
                                         const double dt_eps)
    : Task(robot, 3) {
  myUtils::pretty_constructor(3, "Angular Momentum Task");

  Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
  JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);

  Ag_cur_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
  Ag_prev_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
  Agdot_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
  dt_eps_ = dt_eps;

  // Create flag for the first time we compute Jdotqdot
  first_pass_ = false;
}

AngularMomentumTask::~AngularMomentumTask() {}

bool AngularMomentumTask::_UpdateCommand(const Eigen::VectorXd& _pos_des,
                                         const Eigen::VectorXd& _vel_des,
                                         const Eigen::VectorXd& _acc_des) {
  // Get current Angular Momentum
  Eigen::VectorXd vel_act = robot_->getCentroidMomentum().head(3);

  // Update desired angular momentum and rate
  for (int i = 0; i < dim_task_; ++i) {
    pos_err[i] = 0.0;          // no position error in angular momentum
    vel_des[i] = _vel_des[i];  // desired angular momentum
    acc_des[i] = _acc_des[i];  // desired angular momentum rate feed forward
  }

  // op_cmd
  for (int i(0); i < dim_task_; ++i) {
    op_cmd[i] = acc_des[i] + kd_[i] * (vel_des[i] - vel_act[i]);
  }

  return true;
}

bool AngularMomentumTask::_UpdateCurrent() {
  pos_cur_ = Eigen::VectorXd::Zero(3);
  vel_cur_ = robot_->getCentroidMomentum().head(3);
}

bool AngularMomentumTask::_UpdateTaskJacobian() {
  Eigen::MatrixXd Jtmp = robot_->getCentroidInertiaTimesJacobian();
  Jt_ = Jtmp.block(0, 0, dim_task_, robot_->getNumDofs());

  // Update the previous and current centroidal inertia matrix
  Ag_prev_ = Ag_cur_;
  Ag_cur_ = Jt_;
  return true;
}

bool AngularMomentumTask::_UpdateTaskJDotQdot() {
  // Numerical Approximation
  Agdot_ = (1.0 / dt_eps_) * (Ag_cur_ - Ag_prev_);

  // If we have at least gone through once store the numerical computation
  if (first_pass_) {
    JtDotQdot_ = Agdot_ * robot_->getQdot();
  } else {
    JtDotQdot_.setZero();  // otherwise set the numerical approximation to zero
    first_pass_ = true;    // enable flag
  }

  return true;
}
