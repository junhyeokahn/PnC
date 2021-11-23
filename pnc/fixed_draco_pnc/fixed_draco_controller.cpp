#include "pnc/fixed_draco_pnc/fixed_draco_controller.hpp"

#include "pnc/fixed_draco_pnc/fixed_draco_control_architecture.hpp"
#include "pnc/whole_body_controllers/ihwbc/ihwbc.hpp"
#include "pnc/whole_body_controllers/ihwbc/joint_integrator.hpp"
#include "utils/util.hpp"

FixedDracoController::FixedDracoController(
    FixedDracoTCIContainer *_tci_container, RobotSystem *_robot) {
  util::PrettyConstructor(2, "FixedDracoController");
  tci_container_ = _tci_container;
  robot_ = _robot;

  sp_ = FixedDracoStateProvider::getStateProvider();

  cfg_ = YAML::LoadFile(THIS_COM "config/fixed_draco/pnc.yaml");

  controller_type_ =
      util::ReadParameter<int>(cfg_["controller"], "controller_type");

  // Initialize WBC
  int l_jp_idx = robot_->get_q_dot_idx("l_knee_fe_jp");
  int l_jd_idx = robot_->get_q_dot_idx("l_knee_fe_jd");
  int r_jp_idx = robot_->get_q_dot_idx("r_knee_fe_jp");
  int r_jd_idx = robot_->get_q_dot_idx("r_knee_fe_jd");

  std::vector<bool> act_list;
  for (int i = 0; i < robot_->n_a; ++i)
    act_list.push_back(true);
  act_list[l_jp_idx] = false;
  act_list[r_jp_idx] = false;

  int n_q_dot(act_list.size());
  int n_active(std::count(act_list.begin(), act_list.end(), true));
  int n_passive(n_q_dot - n_active);

  sa_ = Eigen::MatrixXd::Zero(n_active, n_q_dot);
  sv_ = Eigen::MatrixXd::Zero(n_passive, n_q_dot);
  int j(0), k(0);
  for (int i = 0; i < n_q_dot; ++i) {
    if (act_list[i]) {
      sa_(j, i) = 1.;
      j += 1;
    } else {
      sv_(k, i) = 1.;
      k += 1;
    }
  }

  // sf_ can be whatever the random matrix for IHWBC input
  sf_ = Eigen::MatrixXd::Identity(6, 6);

  wbc_ = new IHWBC(sf_, sa_, sv_);
  wbc_->b_trq_limit = util::ReadParameter<bool>(cfg_["wbc"], "b_trq_limit");
  if (wbc_->b_trq_limit) {
    wbc_->trq_limit = sa_.block(0, robot_->n_floating, n_active,
                                n_q_dot - robot_->n_floating) *
                      robot_->joint_trq_limit;
  }
  wbc_->lambda_q_ddot = util::ReadParameter<double>(
      cfg_["wbc"]["regularization"], "lambda_q_ddot");
  wbc_->lambda_rf =
      util::ReadParameter<double>(cfg_["wbc"]["regularization"], "lambda_rf");

  // Initialize Joint Integrator
  joint_integrator_ = new JointIntegrator(robot_->n_a, sp_->servo_dt);
  joint_integrator_->setVelocityFrequencyCutOff(util::ReadParameter<double>(
      cfg_["wbc"]["joint_integrator"], "vel_cutoff_freq"));
  joint_integrator_->setPositionFrequencyCutOff(util::ReadParameter<double>(
      cfg_["wbc"]["joint_integrator"], "pos_cutoff_freq"));
  joint_integrator_->setMaxPositionError(util::ReadParameter<double>(
      cfg_["wbc"]["joint_integrator"], "max_pos_err"));
  joint_integrator_->setVelocityBounds(robot_->joint_vel_limit.col(0),
                                       robot_->joint_vel_limit.col(1));
  joint_integrator_->setPositionBounds(robot_->joint_pos_limit.col(0),
                                       robot_->joint_pos_limit.col(1));

  joint_trq_cmd_ = Eigen::VectorXd::Zero(robot_->n_a);
  joint_vel_cmd_ = Eigen::VectorXd::Zero(robot_->n_a);
  joint_pos_cmd_ = Eigen::VectorXd::Zero(robot_->n_a);

  f_int_ = Eigen::VectorXd::Zero(2);

  b_first_visit_ = true;
  b_smoothing_cmd_ = false;
  smoothing_start_time_ = 0.;
  smoothing_duration_ =
      util::ReadParameter<double>(cfg_["controller"], "smoothing_duration");
  smoothing_start_joint_positions_ = Eigen::VectorXd::Zero(robot_->n_a);
}

FixedDracoController::~FixedDracoController() {
  delete wbc_;
  delete joint_integrator_;
}

void FixedDracoController::getCommand(void *cmd) {
  if (sp_->state == fixed_draco_states::kInitialize) {
    joint_pos_cmd_ = tci_container_->joint_task->pos_des;
    joint_vel_cmd_ = tci_container_->joint_task->vel_des;
    joint_trq_cmd_.setZero();
  } else {
    if (b_first_visit_) {
      FirstVisit();
      b_first_visit_ = false;
    }

    // Dynamics properties
    Eigen::MatrixXd A = robot_->get_mass_matrix();
    Eigen::MatrixXd Ainv = A.inverse();
    Eigen::VectorXd grav = robot_->get_gravity();
    Eigen::VectorXd cori = robot_->get_coriolis();
    wbc_->update_setting(A, Ainv, cori, grav);

    // Task, Contact, Internal Constraint Setup
    wbc_->w_hierarchy = Eigen::VectorXd::Zero(tci_container_->task_list.size());
    for (int i = 0; i < tci_container_->task_list.size(); ++i) {
      tci_container_->task_list[i]->update_jacobian();
      tci_container_->task_list[i]->update_cmd();
      wbc_->w_hierarchy[i] = tci_container_->task_list[i]->w_hierarchy;
    }

    int rf_dim(0);
    for (int i = 0; i < tci_container_->contact_list.size(); ++i) {
      tci_container_->contact_list[i]->update_contact();
      rf_dim += tci_container_->contact_list[i]->dim;
    }

    for (int i = 0; i < tci_container_->internal_constraint_list.size(); ++i) {
      tci_container_->internal_constraint_list[i]->update_internal_constraint();
    }

    // WBC commands
    Eigen::VectorXd rf_des = Eigen::VectorXd::Zero(rf_dim);
    Eigen::VectorXd rf_cmd = Eigen::VectorXd::Zero(rf_dim);
    Eigen::VectorXd wbc_joint_trq_cmd = Eigen::VectorXd::Zero(sa_.rows());
    Eigen::VectorXd wbc_joint_acc_cmd = Eigen::VectorXd::Zero(sa_.rows());
    if (controller_type_ == fixed_draco_controller_type::kGravityCompensation) {
      wbc_joint_trq_cmd =
          this->ComputeGravityCompensationTorques(A, Ainv, grav);
    } else {
      wbc_->solve(tci_container_->task_list, tci_container_->contact_list,
                  tci_container_->internal_constraint_list, rf_des,
                  wbc_joint_trq_cmd, wbc_joint_acc_cmd, rf_cmd, f_int_);
    }
    joint_trq_cmd_ = sa_.transpose() * wbc_joint_trq_cmd;
    Eigen::VectorXd joint_acc_cmd = sa_.transpose() * wbc_joint_acc_cmd;

    joint_integrator_->integrate(joint_acc_cmd, robot_->joint_velocities,
                                 robot_->joint_positions, joint_vel_cmd_,
                                 joint_pos_cmd_);
  }

  if (b_smoothing_cmd_) {
    this->SmoothCommand();
  }

  if (controller_type_ == fixed_draco_controller_type::kGravityCompensation) {
    ((FixedDracoCommand *)cmd)->joint_positions =
        robot_->vector_to_map(robot_->joint_positions);
    ((FixedDracoCommand *)cmd)->joint_velocities =
        robot_->vector_to_map(robot_->joint_velocities);
    ((FixedDracoCommand *)cmd)->joint_torques =
        robot_->vector_to_map(joint_trq_cmd_);

    ((FixedDracoCommand *)cmd)->l_knee_int_frc = f_int_[0];
    ((FixedDracoCommand *)cmd)->r_knee_int_frc = f_int_[1];

  } else if (controller_type_ ==
             fixed_draco_controller_type::kAdmittanceControl) {
    ((FixedDracoCommand *)cmd)->joint_positions =
        robot_->vector_to_map(joint_pos_cmd_);
    ((FixedDracoCommand *)cmd)->joint_velocities =
        robot_->vector_to_map(joint_vel_cmd_);
    ((FixedDracoCommand *)cmd)->joint_torques =
        robot_->vector_to_map(Eigen::VectorXd::Zero(robot_->n_a));

    ((FixedDracoCommand *)cmd)->l_knee_int_frc = 0.;
    ((FixedDracoCommand *)cmd)->r_knee_int_frc = 0.;

  } else if (controller_type_ ==
             fixed_draco_controller_type::kImpedanceControl) {
    ((FixedDracoCommand *)cmd)->joint_positions =
        robot_->vector_to_map(joint_pos_cmd_);
    ((FixedDracoCommand *)cmd)->joint_velocities =
        robot_->vector_to_map(joint_vel_cmd_);
    ((FixedDracoCommand *)cmd)->joint_torques =
        robot_->vector_to_map(joint_trq_cmd_);

    ((FixedDracoCommand *)cmd)->l_knee_int_frc = f_int_[0];
    ((FixedDracoCommand *)cmd)->r_knee_int_frc = f_int_[1];
  }

  // std::cout << "1/2 l_knee_distal torque" << std::endl;
  // std::cout << 0.5*((FixedDracoCommand *)cmd)->joint_torques["l_knee_fe_jd"]
  // << std::endl; std::cout << "1/2 r_knee_distal torque" << std::endl;
  // std::cout << 0.5*((FixedDracoCommand *)cmd)->joint_torques["r_knee_fe_jd"]
  // << std::endl; std::cout << "====================" << std::endl;

  if (sp_->count % sp_->save_freq == 0) {
    this->SaveData();
  }
}

void FixedDracoController::FirstVisit() {
  Eigen::VectorXd jpos_ini = robot_->joint_positions;
  joint_integrator_->initializeStates(Eigen::VectorXd::Zero(robot_->n_a),
                                      jpos_ini);

  b_smoothing_cmd_ =
      util::ReadParameter<bool>(cfg_["controller"], "b_smoothing");
  smoothing_start_time_ = sp_->curr_time;
  smoothing_start_joint_positions_ = robot_->joint_positions;
}

void FixedDracoController::SaveData() {
  FixedDracoDataManager *dm = FixedDracoDataManager::GetFixedDracoDataManager();

  dm->data->cmd_joint_positions = joint_pos_cmd_;
  dm->data->cmd_joint_velocities = joint_vel_cmd_;
  dm->data->cmd_joint_torques = joint_trq_cmd_;

  dm->data->f_int = f_int_;
}

void FixedDracoController::SmoothCommand() {
  double s = util::SmoothPos(0., 1., smoothing_duration_,
                             sp_->curr_time - smoothing_start_time_);

  for (int i = 0; i < robot_->n_a; ++i) {
    joint_pos_cmd_[i] =
        s * joint_pos_cmd_[i] + (1 - s) * smoothing_start_joint_positions_[i];
    joint_vel_cmd_[i] *= s;
    joint_trq_cmd_[i] *= s;
  }
  if (sp_->curr_time >= smoothing_start_time_ + smoothing_duration_) {
    b_smoothing_cmd_ = false;
  }
}

Eigen::VectorXd FixedDracoController::ComputeGravityCompensationTorques(
    const Eigen::MatrixXd &mass, const Eigen::MatrixXd &mass_inv,
    const Eigen::VectorXd &grav) {

  Eigen::MatrixXd jac_i = tci_container_->internal_constraint_list[0]->jacobian;
  Eigen::MatrixXd lambda_i =
      util::PseudoInverse(jac_i * mass_inv * jac_i.transpose(), 0.00001);
  Eigen::MatrixXd jac_i_bar = mass_inv * jac_i.transpose() * lambda_i;
  Eigen::MatrixXd null_i =
      Eigen::MatrixXd::Identity(robot_->n_q_dot, robot_->n_q_dot) -
      jac_i_bar * jac_i;

  Eigen::MatrixXd lmd_sa_ni = util::PseudoInverse(
      (sa_ * null_i) * mass_inv * (sa_ * null_i).transpose(), 0.00001);
  // dyn pseudo
  Eigen::MatrixXd sa_ni_bar = mass_inv * (sa_ * null_i).transpose() * lmd_sa_ni;

  Eigen::MatrixXd sa_ni_bar_trns = sa_ni_bar.transpose();

  // f internal force calulation
  f_int_ = jac_i_bar.transpose() * (grav - sa_.transpose() * sa_ni_bar_trns *
                                               null_i.transpose() * grav);

  return sa_ni_bar_trns * null_i.transpose() * grav;
}
