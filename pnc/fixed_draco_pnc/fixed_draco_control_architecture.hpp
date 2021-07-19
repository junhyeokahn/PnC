#pragma once

#include "configuration.hpp"
#include "pnc/control_architecture.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_controller.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_data_manager.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_tci_container.hpp"
#include "pnc/robot_system/robot_system.hpp"
#include "pnc/whole_body_controllers/managers/foot_trajectory_manager.hpp"
#include "pnc/whole_body_controllers/managers/reaction_force_manager.hpp"
#include "pnc/whole_body_controllers/managers/task_hierarchy_manager.hpp"
#include "pnc/whole_body_controllers/managers/upper_body_trajectory_manager.hpp"

namespace draco_states {
constexpr int kInitialize = 0;
constexpr int kHold = 1;
constexpr int kSwaying = 2;
} // namespace draco_states

class FixedDracoControlArchitecture : public ControlArchitecture {
public:
  FixedDracoControlArchitecture(RobotSystem *_robot);
  virtual ~FixedDracoControlArchitecture();

  virtual void getCommand(void *_command);

  FootTrajectoryManager *rfoot_tm;
  FootTrajectoryManager *lfoot_tm;
  UpperBodyTrajectoryManager *upper_body_tm;
  TaskHierarchyManager *rfoot_pos_hm;
  TaskHierarchyManager *rfoot_ori_hm;
  TaskHierarchyManager *lfoot_pos_hm;
  TaskHierarchyManager *lfoot_ori_hm;

  FixedDracoTCIContainer *tci_container;

private:
  /* data */
  RobotSystem *robot_;

  FixedDracoController *controller_;

  DracoStateProvider *sp_;

  void SaveData();
};
