#include "pnc/draco_pnc/draco_rolling_joint_constraint.hpp"

DracoRollingJointConstraint::DracoRollingJointConstraint(RobotSystem *_robot)
    : InternalConstraint(_robot, 2) {
  int l_jp_idx = robot_->get_q_dot_idx("l_knee_fe_jp");
  int l_jd_idx = robot_->get_q_dot_idx("l_knee_fe_jd");
  int r_jp_idx = robot_->get_q_dot_idx("r_knee_fe_jp");
  int r_jd_idx = robot_->get_q_dot_idx("r_knee_fe_jd");

  jacobian(0, l_jp_idx) = 1.;
  jacobian(0, l_jd_idx) = -1.;
  jacobian(1, r_jp_idx) = 1.;
  jacobian(1, r_jd_idx) = -1.;
  jacobian_dot_q_dot = Eigen::VectorXd::Zero(dim_);
}
