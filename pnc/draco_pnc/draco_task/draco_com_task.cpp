#include "pnc/draco_pnc/draco_task/draco_com_task.hpp"

DracoCenterOfMassTask::DracoCenterOfMassTask(RobotSystem *_robot,
                                             int _feedback_source,
                                             int _feedback_height_target)
    : Task(_robot, 3) {

  util::PrettyConstructor(3, "DracoCenterOfMassTask");
  sp_ = DracoStateProvider::getStateProvider();
  feedback_source_ = _feedback_source;
  feedback_height_target_ = _feedback_height_target;
  icp_des.setZero();
  icp_dot_des.setZero();

  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");

  double time_constant =
      util::ReadParameter<double>(cfg["wbc"]["task"]["icp"], "time_constant");

  Eigen::VectorXd icp_err_lim = Eigen::VectorXd::Zero(2);
  icp_err_lim << 0.03, 0.03;
  icp_err_integrator_ = new ExponentialMovingAverageFilter(
      sp_->servo_dt, time_constant, Eigen::VectorXd::Zero(2), -icp_err_lim,
      icp_err_lim);
}

void DracoCenterOfMassTask::update_cmd() {

  if (feedback_source_ == feedback_source::kCom) {
    pos = robot_->get_com_pos();
    if (feedback_height_target_ == feedback_height_target::kBaseHeight) {
      pos[2] = robot_->get_link_iso("torso_com_link").translation()[2];
    }
    pos_err = pos_des - pos;
    vel = sp_->com_vel_est;
    if (feedback_height_target_ == feedback_height_target::kBaseHeight) {
      vel[2] = robot_->get_link_vel("torso_com_link")[5];
    }

    rot_world_local_ =
        robot_->get_link_iso(robot_->get_base_link_name()).linear();

    local_pos_err = rot_world_local_.transpose() * pos_err;
    local_pos_err[2] = pos_des[2] - pos[2]; // assume z axis is upright
    local_vel_err = rot_world_local_.transpose() * (vel_des - vel);

    op_cmd = acc_des + rot_world_local_ * (kp.cwiseProduct(local_pos_err) +
                                           kd.cwiseProduct(local_vel_err));
  } else if (feedback_source_ == feedback_source::kIcp) {

    Eigen::Vector3d com_pos = robot_->get_com_pos();
    Eigen::Vector3d com_vel = sp_->com_vel_est;
    Eigen::Vector2d icp = sp_->dcm.head(2);

    double z_des(pos_des[2]);
    double z_dot_des(vel_des[2]);
    double omega(sqrt(9.81 / z_des));
    // double omega(sqrt(9.81 / com_pos[2]));

    icp_des = pos_des.head(2) + vel_des.head(2) / omega;
    icp_dot_des = vel_des.head(2) + acc_des.head(2) / omega;

    icp_err_integrator_->Input(icp_des - icp);

    pos = com_pos;
    if (feedback_height_target_ == feedback_height_target::kBaseHeight) {
      pos[2] = robot_->get_link_iso("torso_com_link").translation()[2];
    }
    vel = com_vel;
    if (feedback_height_target_ == feedback_height_target::kBaseHeight) {
      vel[2] = robot_->get_link_vel("torso_com_link")[5];
    }
    pos_err = pos_des - pos;

    rot_world_local_ =
        robot_->get_link_iso(robot_->get_base_link_name()).linear();

    local_pos_err = rot_world_local_.transpose() * pos_err;
    local_pos_err[2] = pos_des[2] - pos[2]; // assume z axis is upright
    local_vel_err = rot_world_local_.transpose() * (vel_des - vel);

    Eigen::Vector2d cmp_des =
        icp - icp_dot_des / omega - kp.head(2).cwiseProduct(icp_des - icp) -
        ki.head(2).cwiseProduct(icp_err_integrator_->Output());

    op_cmd.head(2) = (9.81 / z_des) * (com_pos.head(2) - cmp_des);
    op_cmd[2] = kp[2] * (z_des - pos[2]) + kd[2] * (z_dot_des - vel[2]);

  } else {
    assert(false);
  }
}

void DracoCenterOfMassTask::update_jacobian() {
  if (feedback_height_target_ == feedback_height_target::kBaseHeight) {

    jacobian.block(0, 0, 2, robot_->n_q_dot) =
        robot_->get_com_lin_jacobian().block(0, 0, 2, robot_->n_q_dot);
    jacobian.block(2, 0, 1, robot_->n_q_dot) =
        robot_->get_link_jacobian("torso_com_link")
            .block(5, 0, 1, robot_->n_q_dot);
    jacobian_dot_q_dot.head(2) =
        (robot_->get_com_lin_jacobian_dot() * robot_->get_q_dot()).head(2);
    jacobian_dot_q_dot.tail(1) =
        (robot_->get_link_jacobian_dot_times_qdot("torso_com_link")).tail(1);

  } else if (feedback_height_target_ == feedback_height_target::kComHeight) {
    jacobian = robot_->get_com_lin_jacobian();
    jacobian_dot_q_dot =
        robot_->get_com_lin_jacobian_dot() * robot_->get_q_dot();
  }
}
