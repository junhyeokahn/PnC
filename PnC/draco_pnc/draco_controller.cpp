#include "PnC/draco_pnc/draco_controller.hpp"

#include "PnC/WBC/IHWBC/IHWBC.hpp"
#include "PnC/WBC/IHWBC/JointIntegrator.hpp"
#include "PnC/draco_pnc/draco_control_architecture.hpp"
#include "Utils/IO/DataManager.hpp"

DracoController::DracoController(DracoTCIContainer *_tci_container,
                                 RobotSystem *_robot) {
  tci_container_ = _tci_container;
  robot_ = _robot;

  sp_ = DracoStateProvider::getStateProvider();

  YAML::Node cfg = YAML::LoadFile(THIS_COM "Config/draco/pnc.yaml");

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
  act_list[l_jd_idx] = false;
  act_list[r_jd_idx] = false;

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

  wbc_ = new IHWBC(sf_, sa_, sv_);
  wbc_->b_trq_limit = myUtils::readParameter<bool>(cfg["wbc"], "b_trq_limit");
  if (wbc_->b_trq_limit) {
    wbc_->trq_limit = sa_.block(0, robot_->n_floating, n_active,
                                n_q_dot - robot_->n_floating) *
                      robot_->joint_trq_limit;
  }
  wbc_->lambda_q_ddot =
      myUtils::readParameter<double>(cfg["wbc"], "lambda_q_ddot");
  wbc_->lambda_rf = myUtils::readParameter<double>(cfg["wbc"], "lambda_rf");

  // Initialize Joint Integrator
  joint_integrator_ = new JointIntegrator(robot_->n_a, sp_->servo_rate);
  joint_integrator_->setVelocityFrequencyCutOff(
      myUtils::readParameter<double>(cfg["wbc"], "vel_cutoff_freq"));
  joint_integrator_->setPositionFrequencyCutOff(
      myUtils::readParameter<double>(cfg["wbc"], "pos_cutoff_freq"));
  joint_integrator_->setMaxPositionError(
      myUtils::readParameter<double>(cfg["wbc"], "max_pos_err"));
  joint_integrator_->setVelocityBounds(robot_->joint_vel_limit.col(0),
                                       robot_->joint_vel_limit.col(1));
  joint_integrator_->setPositionBounds(robot_->joint_pos_limit.col(0),
                                       robot_->joint_pos_limit.col(1));

  joint_trq_cmd_ = Eigen::VectorXd::Zero(robot_->n_a);
  joint_vel_cmd_ = Eigen::VectorXd::Zero(robot_->n_a);
  joint_pos_cmd_ = Eigen::VectorXd::Zero(robot_->n_a);
  r_rf_cmd_ = Eigen::VectorXd::Zero(6);
  l_rf_cmd_ = Eigen::VectorXd::Zero(6);

  DataManager *data_manager = DataManager::GetDataManager();
  data_manager->RegisterData(&joint_pos_cmd_, VECT, "cmd_jpos", robot_->n_a);
  data_manager->RegisterData(&joint_vel_cmd_, VECT, "cmd_jvel", robot_->n_a);
  data_manager->RegisterData(&joint_trq_cmd_, VECT, "cmd_jtrq", robot_->n_a);
  data_manager->RegisterData(&r_rf_cmd_, VECT, "cmd_r_rf", 6);
  data_manager->RegisterData(&l_rf_cmd_, VECT, "cmd_l_rf", 6);
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

  } else {
    static bool b_first_visit(true);
    if (b_first_visit) {
      FirstVisit();
      b_first_visit = false;
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
    Eigen::VectorXd wbc_joint_trq_cmd = Eigen::VectorXd::Zero(sa_.rows());
    Eigen::VectorXd wbc_joint_acc_cmd = Eigen::VectorXd::Zero(sa_.rows());
    wbc_->solve(tci_container_->task_list, tci_container_->contact_list,
                tci_container_->internal_constraint_list, rf_des,
                wbc_joint_trq_cmd, wbc_joint_acc_cmd, rf_des);
    joint_trq_cmd_ = (sa_.block(0, 6, sa_.rows(), sa_.cols() - 6)).transpose() *
                     wbc_joint_trq_cmd;
    Eigen::VectorXd joint_acc_cmd =
        (sa_.block(0, 6, sa_.rows(), sa_.cols() - 6)).transpose() *
        wbc_joint_acc_cmd;
    if (sp_->state == draco_states::kLFootSwing) {
      l_rf_cmd_ = Eigen::VectorXd::Zero(6);
      r_rf_cmd_ = rf_des;
    } else if (sp_->state == draco_states::kRFootSwing) {
      r_rf_cmd_ = Eigen::VectorXd::Zero(6);
      l_rf_cmd_ = rf_des;
    } else {
      // right foot first
      r_rf_cmd_ = rf_des.head(6);
      l_rf_cmd_ = rf_des.tail(6);
    }

    joint_integrator_->integrate(joint_acc_cmd, robot_->joint_velocities,
                                 robot_->joint_positions, joint_vel_cmd_,
                                 joint_pos_cmd_);
  }

  ((DracoCommand *)cmd)->joint_positions =
      robot_->vector_to_map(joint_pos_cmd_);
  ((DracoCommand *)cmd)->joint_velocities =
      robot_->vector_to_map(joint_vel_cmd_);
  ((DracoCommand *)cmd)->joint_torques = robot_->vector_to_map(joint_trq_cmd_);
}

void DracoController::FirstVisit() {
  Eigen::VectorXd jpos_ini = robot_->joint_positions;
  joint_integrator_->initializeStates(Eigen::VectorXd::Zero(robot_->n_a),
                                      jpos_ini);
}
