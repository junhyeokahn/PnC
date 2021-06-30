#pragma once

#include "PnC/RobotSystem/RobotSystem.hpp"
#include "PnC/WBC/IHWBC/IHWBC.hpp"
#include "PnC/WBC/IHWBC/JointIntegrator.hpp"
#include "PnC/draco_pnc/draco_interface.hpp"
#include "PnC/draco_pnc/draco_state_provider.hpp"
#include "PnC/draco_pnc/draco_tci_container.hpp"

class DracoController {
public:
  DracoController(DracoTCIContainer *_tci_container, RobotSystem *_robot);
  virtual ~DracoController();

  void getCommand(void *_cmd);

private:
  /* data */
  RobotSystem *robot_;
  IHWBC *wbc_;
  JointIntegrator *joint_integrator_;

  DracoTCIContainer *tci_container_;
  DracoStateProvider *sp_;

  Eigen::VectorXd joint_trq_cmd_;
  Eigen::VectorXd joint_acc_cmd_;
  Eigen::VectorXd r_rf_cmd_;
  Eigen::VectorXd l_rf_cmd_;
  Eigen::VectorXd joint_pos_cmd_;
  Eigen::VectorXd joint_vel_cmd_;

  void FirstVisit();

  Eigen::MatrixXd sa_;
  Eigen::MatrixXd sv_;
  Eigen::MatrixXd sf_;
};
