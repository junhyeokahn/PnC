#pragma once

#include "pnc/fixed_draco_pnc/fixed_draco_interface.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_tci_container.hpp"
#include "pnc/robot_system/robot_system.hpp"
#include "pnc/whole_body_controllers/ihwbc/ihwbc.hpp"
#include "pnc/whole_body_controllers/ihwbc/joint_integrator.hpp"

class FixedDracoController {
public:
  FixedDracoController(FixedDracoTCIContainer *_tci_container, RobotSystem *_robot);
  virtual ~FixedDracoController();

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

  void FirstVisit();

  Eigen::MatrixXd sa_;
  Eigen::MatrixXd sv_;
  Eigen::MatrixXd sf_;
};
