#pragma once

#include "configuration.hpp"
#include "pnc/control_architecture.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_controller.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_data_manager.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_tci_container.hpp"
#include "pnc/robot_system/robot_system.hpp"
#include "pnc/whole_body_controllers/managers/end_effector_trajectory_manager.hpp"
#include "pnc/whole_body_controllers/managers/upper_body_trajectory_manager.hpp"

namespace fixed_draco_states {
constexpr int kInitialize = 0;
constexpr int kHold = 1;
constexpr int kRightFootSwaying = 2;
constexpr int kLeftFootSwaying = 3;
} // namespace fixed_draco_states

class FixedDracoControlArchitecture : public ControlArchitecture {
public:
  FixedDracoControlArchitecture(RobotSystem *_robot);
  virtual ~FixedDracoControlArchitecture();

  virtual void getCommand(void *_command);

  EndEffectorTrajectoryManager *rf_ee_tm;
  EndEffectorTrajectoryManager *lf_ee_tm;
  UpperBodyTrajectoryManager *upper_body_tm;

  FixedDracoTCIContainer *tci_container;

private:
  /* data */
  RobotSystem *robot_;

  FixedDracoController *controller_;

  FixedDracoStateProvider *sp_;

  void SaveData();
};
