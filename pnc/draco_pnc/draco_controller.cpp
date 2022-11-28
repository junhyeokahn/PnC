#include "pnc/draco_pnc/draco_controller.hpp"

#include "pnc/draco_pnc/draco_control_architecture.hpp"
#include "pnc/whole_body_controllers/ihwbc/ihwbc.hpp"
#include "pnc/whole_body_controllers/ihwbc/joint_integrator.hpp"

DracoController::DracoController(DracoTCIContainer *_tci_container,
                                 RobotSystem *_robot) {
  util::PrettyConstructor(2, "DracoController");
  tci_container_ = _tci_container;
  robot_ = _robot;

  sp_ = DracoStateProvider::getStateProvider();

  cfg_ = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");

  // Initialize WBC
  int l_jp_idx = robot_->get_q_dot_idx("l_knee_fe_jp");
  int l_jd_idx = robot_->get_q_dot_idx("l_knee_fe_jd");
  int r_jp_idx = robot_->get_q_dot_idx("r_knee_fe_jp");
  int r_jd_idx = robot_->get_q_dot_idx("r_knee_fe_jd");

  std::vector<bool> act_list;
  for (int i = 0; i < robot_->n_floating; ++i)
    act_list.push_back(false);
  for (int i = 0; i < robot_->n_a; ++i)
    act_list.push_back(true);
  act_list[l_jp_idx] = false;
  act_list[r_jp_idx] = false;

  int n_q_dot(act_list.size());
  int n_active(std::count(act_list.begin(), act_list.end(), true));
  int n_passive(n_q_dot - n_active - robot_->n_floating);

  sa_ = Eigen::MatrixXd::Zero(n_active, n_q_dot);
  sv_ = Eigen::MatrixXd::Zero(n_passive, n_q_dot);
  int j(0), k(0);
  for (int i = 0; i < n_q_dot; ++i) {
    if (i >= 6) {
      if (act_list[i]) {
        sa_(j, i) = 1.;
        j += 1;
      } else {
        sv_(k, i) = 1.;
        k += 1;
      }
    }
  }
  sf_ = Eigen::MatrixXd::Zero(robot_->n_floating, n_q_dot);
  sf_.block(0, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
  snf_ = Eigen::MatrixXd::Zero(n_q_dot - robot_->n_floating, n_q_dot);
  snf_.rightCols(n_q_dot - robot_->n_floating) = Eigen::MatrixXd::Identity(
      n_q_dot - robot_->n_floating, n_q_dot - robot_->n_floating);

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
  wbc_->lambda_rf_new = util::ReadParameter<Eigen::VectorXd>(
      cfg_["wbc"]["regularization"], "lambda_rf_new");
  wbc_->w_rf = util::ReadParameter<double>(cfg_["wbc"]["contact"], "w_rf");

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
  joint_acc_cmd_ = Eigen::VectorXd::Zero(robot_->n_a);
  joint_acc_cmd_fb_ = Eigen::VectorXd::Zero(6);
  cmd_rfoot_rf_ = Eigen::VectorXd::Zero(6);
  cmd_lfoot_rf_ = Eigen::VectorXd::Zero(6);

  l_knee_int_frc_cmd_ = 0.;
  r_knee_int_frc_cmd_ = 0.;

  b_first_visit_ = true;
  b_smoothing_cmd_ = false;
  smoothing_start_time_ = 0.;
  smoothing_duration_ =
      util::ReadParameter<double>(cfg_["controller"], "smoothing_duration");
  smoothing_start_joint_positions_ = Eigen::VectorXd::Zero(robot_->n_a);

  b_use_modified_swing_jac_ =
      util::ReadParameter<bool>(cfg_["controller"], "use_modified_swing_jac");
}

DracoController::~DracoController() {
  delete wbc_;
  delete joint_integrator_;
}

void DracoController::getCommand(void *cmd) {
  if (sp_->state == draco_states::kInitialize) {
    joint_pos_cmd_ = tci_container_->joint_task->pos_des;
    joint_vel_cmd_ = tci_container_->joint_task->vel_des;
    joint_trq_cmd_.setZero();

    l_knee_int_frc_cmd_ = 0.;
    r_knee_int_frc_cmd_ = 0.;

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
    for (int i = 0; i < tci_container_->task_list.size(); ++i) {
      tci_container_->task_list[i]->update_jacobian();
      // tci_container_->task_list[i]->update_cmd(
      // sp_->nominal_stance_foot_iso.linear());
      tci_container_->task_list[i]->update_cmd(
          sp_->nominal_base_quat.toRotationMatrix());
    }

    if (b_use_modified_swing_jac_) {
      // ignore jacobian cols depending on the state
      if (!sp_->b_rf_contact) {
        // ignore right foot joints for com task
        // tci_container_->task_list[0]->ignore_jacobian_row(sp_->rfoot_jidx);
        // tci_container_->task_list[2]->ignore_jacobian_row(sp_->rfoot_jidx);
        // ignore floating joints for swing foot task
        tci_container_->task_list[4]->ignore_jacobian_row(sp_->floating_jidx);
        tci_container_->task_list[5]->ignore_jacobian_row(sp_->floating_jidx);
      }
      if (!sp_->b_lf_contact) {
        // ignore left foot joints for com task
        // tci_container_->task_list[0]->ignore_jacobian_row(sp_->lfoot_jidx);
        // tci_container_->task_list[2]->ignore_jacobian_row(sp_->lfoot_jidx);
        // ignore floating joints for swing foot task
        tci_container_->task_list[6]->ignore_jacobian_row(sp_->floating_jidx);
        tci_container_->task_list[7]->ignore_jacobian_row(sp_->floating_jidx);
      }
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
    Eigen::VectorXd wbc_joint_trq_cmd = Eigen::VectorXd::Zero(sa_.rows());
    Eigen::VectorXd wbc_joint_acc_cmd = Eigen::VectorXd::Zero(robot_->n_q_dot);
    Eigen::VectorXd wbc_int_frc_cmd;
    wbc_->solve(tci_container_->task_list, tci_container_->contact_list,
                tci_container_->internal_constraint_list, rf_des,
                wbc_joint_trq_cmd, wbc_joint_acc_cmd, rf_des, wbc_int_frc_cmd);
    l_knee_int_frc_cmd_ = wbc_int_frc_cmd[0];
    r_knee_int_frc_cmd_ = wbc_int_frc_cmd[1];
    joint_trq_cmd_ =
        sa_.rightCols(sa_.cols() - sf_.rows()).transpose() * wbc_joint_trq_cmd;
    joint_acc_cmd_ = snf_ * wbc_joint_acc_cmd;
    joint_acc_cmd_fb_ = sf_ * wbc_joint_acc_cmd;

    // if (sp_->state == draco_states::kLFootSwing) {
    // cmd_lfoot_rf_ = Eigen::VectorXd::Zero(6);
    // cmd_rfoot_rf_ = rf_des.head(6);
    //} else if (sp_->state == draco_states::kRFootSwing) {
    // cmd_rfoot_rf_ = Eigen::VectorXd::Zero(6);
    // cmd_lfoot_rf_ = rf_des.tail(6);
    //} else {
    // cmd_rfoot_rf_ = rf_des.head(6);
    // cmd_lfoot_rf_ = rf_des.tail(6);
    //}

    // right foot first
    cmd_rfoot_rf_ = rf_des.head(6);
    cmd_lfoot_rf_ = rf_des.tail(6);

    joint_integrator_->integrate(joint_acc_cmd_, robot_->joint_velocities,
                                 robot_->joint_positions, joint_vel_cmd_,
                                 joint_pos_cmd_);
  }

  if (b_smoothing_cmd_) {
    this->SmoothCommand();
  }

  ((DracoCommand *)cmd)->joint_positions =
      robot_->vector_to_map(joint_pos_cmd_);
  ((DracoCommand *)cmd)->joint_velocities =
      robot_->vector_to_map(joint_vel_cmd_);
  ((DracoCommand *)cmd)->joint_torques = robot_->vector_to_map(joint_trq_cmd_);

  ((DracoCommand *)cmd)->l_knee_int_frc = l_knee_int_frc_cmd_;
  ((DracoCommand *)cmd)->r_knee_int_frc = r_knee_int_frc_cmd_;

  if (sp_->count % sp_->save_freq == 0) {
    this->SaveData();
  }
}

void DracoController::FirstVisit() {
  Eigen::VectorXd jpos_ini = robot_->joint_positions;
  joint_integrator_->initializeStates(Eigen::VectorXd::Zero(robot_->n_a),
                                      jpos_ini);

  b_smoothing_cmd_ =
      util::ReadParameter<bool>(cfg_["controller"], "b_smoothing");
  smoothing_start_time_ = sp_->curr_time;
  smoothing_start_joint_positions_ = robot_->joint_positions;
}

void DracoController::SmoothCommand() {
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

void DracoController::SaveData() {
  DracoDataManager *dm = DracoDataManager::GetDracoDataManager();

  dm->data->cmd_lfoot_rf = cmd_lfoot_rf_;
  dm->data->cmd_rfoot_rf = cmd_rfoot_rf_;

  dm->data->cmd_joint_positions = joint_pos_cmd_;
  dm->data->cmd_joint_velocities = joint_vel_cmd_;
  dm->data->cmd_joint_torques = joint_trq_cmd_;
  dm->data->cmd_joint_accelerations = joint_acc_cmd_;
  dm->data->cmd_joint_accelerations_fb = joint_acc_cmd_fb_;

  dm->data->l_knee_int_frc_cmd = l_knee_int_frc_cmd_;
  dm->data->r_knee_int_frc_cmd = r_knee_int_frc_cmd_;
}
