#pragma once

#include <configuration.hpp>
#include <pnc/atlas_pnc/atlas_controller.hpp>
#include <pnc/atlas_pnc/atlas_tci_container.hpp>
#include <pnc/control_architecture.hpp>
#include <pnc/planners/locomotion/dcm_planner/dcm_planner.hpp>
#include <pnc/robot_system/robot_system.hpp>
#include <pnc/whole_body_controllers/managers/dcm_trajectory_manager.hpp>
#include <pnc/whole_body_controllers/managers/floating_base_trajectory_manager.hpp>
#include <pnc/whole_body_controllers/managers/foot_trajectory_manager.hpp>
#include <pnc/whole_body_controllers/managers/reaction_force_manager.hpp>
#include <pnc/whole_body_controllers/managers/task_hierarchy_manager.hpp>
#include <pnc/whole_body_controllers/managers/upper_body_trajectory_manager.hpp>

namespace AtlasStates {
constexpr int Stand = 0;
constexpr int Balance = 1;
constexpr int LFootContactTransitionStart = 2;
constexpr int LFootContactTransitionEnd = 3;
constexpr int LFootSwing = 4;
constexpr int RFootContactTransitionStart = 5;
constexpr int RFootContactTransitionEnd = 6;
constexpr int RFootSwing = 7;
} // namespace AtlasStates

class AtlasControlArchitecture : public ControlArchitecture {
public:
  AtlasControlArchitecture(RobotSystem *_robot);
  virtual ~AtlasControlArchitecture();

  virtual void getCommand(void *_command);

  FootTrajectoryManager *rfoot_tm;
  FootTrajectoryManager *lfoot_tm;
  UpperBodyTrajectoryManager *upper_body_tm;
  FloatingBaseTrajectoryManager *floating_base_tm;
  DCMTrajectoryManager *dcm_tm;
  TaskHierarchyManager *rfoot_pos_hm;
  TaskHierarchyManager *rfoot_ori_hm;
  TaskHierarchyManager *lfoot_pos_hm;
  TaskHierarchyManager *lfoot_ori_hm;
  ReactionForceManager *rfoot_fm;
  ReactionForceManager *lfoot_fm;

private:
  /* data */
  RobotSystem *robot_;
  DCMPlanner *dcm_planner_;

  AtlasController *controller_;
  AtlasTCIContainer *tci_container_;

  AtlasStateProvider *sp_;
};
