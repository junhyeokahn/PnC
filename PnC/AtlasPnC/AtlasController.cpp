#include <PnC/AtlasPnC/AtlasController.hpp>
#include <PnC/WBC/IHWBC/IHWBC.hpp>
#include <PnC/WBC/IHWBC/JointIntegrator.hpp>

AtlasController::AtlasController(AtlasTCIContainer *_tci_container,
                                 RobotSystem *_robot) {
  tci_container_ = _tci_container;
  robot_ = _robot;

  sp_ = AtlasStateProvider::getStateProvider(robot_);

  YAML::Node cfg = YAML::LoadFile(THIS_COM "Config/Atlas/pnc.yaml");

  // Initialize WBC
  std::vector<bool> act_list;
  for (int i = 0; i < robot_->n_floating; ++i)
    act_list.push_back(false);
  for (int i = 0; i < robot_->n_a; ++i)
    act_list.push_back(true);
  int n_q_dot(act_list.size());
  int n_active(robot_->n_a);
  int n_passive(n_q_dot - n_active - robot_->n_floating);

  Eigen::MatrixXd sa = Eigen::MatrixXd::Zero(n_active, n_q_dot);
  Eigen::MatrixXd sv = Eigen::MatrixXd::Zero(n_passive, n_q_dot);
  int j(0), k(0);
  for (int i = 0; i < n_q_dot; ++i) {
    if (i >= 6) {
      if (act_list[i]) {
        sa(j, i) = 1.;
        j += 1;
      } else {
        sv(k, i) = 1.;
        k += 1;
      }
    }
  }
  Eigen::MatrixXd sf = Eigen::MatrixXd::Zero(robot_->n_floating, n_q_dot);
  sf.block(0, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
  wbc_ = new IHWBC(sf, sa, sv);
  wbc_->b_trq_limit = myUtils::readParameter<bool>(cfg["wbc"], "b_trq_limit");
  if (wbc_->b_trq_limit) {
    wbc_->trq_limit = sa.block(0, robot_->n_floating, n_active,
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
}

AtlasController::~AtlasController() {
  delete wbc_;
  delete joint_integrator_;
}

void AtlasController::getCommand(void *cmd) {
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
  Eigen::VectorXd joint_trq_cmd, joint_acc_cmd, rf_cmd;
  wbc_->solve(tci_container_->task_list, tci_container_->contact_list,
              tci_container_->internal_constraint_list, rf_des, joint_trq_cmd,
              joint_acc_cmd, rf_cmd);
  Eigen::VectorXd joint_vel_cmd = Eigen::VectorXd::Zero(robot_->n_a);
  Eigen::VectorXd joint_pos_cmd = Eigen::VectorXd::Zero(robot_->n_a);
  // joint_integrator_->integrate(joint_acc_cmd, robot_->joint_velocities,
  // robot_->joint_positions, joint_vel_cmd,
  // joint_pos_cmd);

  ((AtlasCommand *)cmd)->joint_positions =
      robot_->create_cmd_map(joint_pos_cmd);
  ((AtlasCommand *)cmd)->joint_velocities =
      robot_->create_cmd_map(joint_vel_cmd);
  ((AtlasCommand *)cmd)->joint_torques = robot_->create_cmd_map(joint_trq_cmd);
}

void AtlasController::FirstVisit() {
  Eigen::VectorXd jpos_ini = robot_->joint_positions;
  // joint_integrator_->initializeStates(Eigen::VectorXd::Zero(robot_->n_a),
  // jpos_ini);
}
