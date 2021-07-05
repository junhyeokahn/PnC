#pragma once

#include <pnc/atlas_pnc/atlas_interface.hpp>
#include <pnc/atlas_pnc/atlas_state_provider.hpp>
#include <pnc/atlas_pnc/atlas_tci_container.hpp>
#include <pnc/robot_system/robot_system.hpp>
#include <pnc/whole_body_controllers/ihwbc/ihwbc.hpp>
#include <pnc/whole_body_controllers/ihwbc/joint_integrator.hpp>

class AtlasController {
public:
  AtlasController(AtlasTCIContainer *_tci_container, RobotSystem *_robot);
  virtual ~AtlasController();

  void getCommand(void *_cmd);

private:
  /* data */
  RobotSystem *robot_;
  IHWBC *wbc_;
  JointIntegrator *joint_integrator_;

  AtlasTCIContainer *tci_container_;
  AtlasStateProvider *sp_;

  Eigen::VectorXd joint_trq_cmd_;
  Eigen::VectorXd joint_acc_cmd_;
  Eigen::VectorXd r_rf_cmd_;
  Eigen::VectorXd l_rf_cmd_;
  Eigen::VectorXd joint_pos_cmd_;
  Eigen::VectorXd joint_vel_cmd_;

  void FirstVisit();
};
