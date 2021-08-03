#pragma once

#include "pnc/fixed_draco_pnc/fixed_draco_interface.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_tci_container.hpp"
#include "pnc/robot_system/robot_system.hpp"
#include "pnc/whole_body_controllers/ihwbc/ihwbc.hpp"
#include "pnc/whole_body_controllers/ihwbc/joint_integrator.hpp"

namespace fixed_draco_controller_type {
constexpr int kGravityCompensation = 0;
constexpr int kAdmittanceControl = 1;
constexpr int kImpedanceControl = 2;
} // namespace fixed_draco_controller_type

class FixedDracoController {
public:
  FixedDracoController(FixedDracoTCIContainer *_tci_container,
                       RobotSystem *_robot);
  virtual ~FixedDracoController();

  double smoothing_duration;

  void getCommand(void *_cmd);

  void SaveData();

private:
  /* data */
  RobotSystem *robot_;
  IHWBC *wbc_;
  JointIntegrator *joint_integrator_;

  FixedDracoTCIContainer *tci_container_;
  FixedDracoStateProvider *sp_;

  Eigen::VectorXd joint_trq_cmd_;
  Eigen::VectorXd joint_acc_cmd_;
  Eigen::VectorXd joint_pos_cmd_;
  Eigen::VectorXd joint_vel_cmd_;

  Eigen::MatrixXd sa_;
  Eigen::MatrixXd sv_;
  Eigen::MatrixXd sf_;

  bool b_smoothing_cmd_;
  double smoothing_start_time_;
  Eigen::VectorXd smoothing_start_joint_positions_;

  // First visit of IHWBC
  void FirstVisit();

  // Smooth increment of the commands
  void SmoothCommand();

  // compute gravity compensation torques
  Eigen::VectorXd
  ComputeGravityCompensationTorques(const Eigen::MatrixXd &mass,
                                    const Eigen::MatrixXd &mass_inv,
                                    const Eigen::VectorXd &grav);

  int controller_type_;
  YAML::Node cfg_;

  bool b_first_visit_;
};
