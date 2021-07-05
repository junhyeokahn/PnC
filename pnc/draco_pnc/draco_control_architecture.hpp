#pragma once

#include <configuration.hpp>
#include <pnc/control_architecture.hpp>
#include <pnc/planners/locomotion/dcm_planner/dcm_planner.hpp>
#include <pnc/robot_system/robot_system.hpp>
#include <pnc/whole_body_controllers/managers/dcm_trajectory_manager.hpp>
#include <pnc/whole_body_controllers/managers/floating_base_trajectory_manager.hpp>
#include <pnc/whole_body_controllers/managers/foot_trajectory_manager.hpp>
#include <pnc/whole_body_controllers/managers/reaction_force_manager.hpp>
#include <pnc/whole_body_controllers/managers/task_hierarchy_manager.hpp>
#include <pnc/whole_body_controllers/managers/upper_body_trajectory_manager.hpp>
#include <pnc/draco_pnc/draco_controller.hpp>
#include <pnc/draco_pnc/draco_tci_container.hpp>

namespace draco_states {
constexpr int kInitialize = 0;
constexpr int kStand = 1;
constexpr int kBalance = 2;
constexpr int kLFootContactTransitionStart = 3;
constexpr int kLFootContactTransitionEnd = 4;
constexpr int kLFootSwing = 5;
constexpr int kRFootContactTransitionStart = 6;
constexpr int kRFootContactTransitionEnd = 7;
constexpr int kRFootSwing = 8;
constexpr int kSwaying = 10;
} // namespace draco_states

class DracoControlArchitecture : public ControlArchitecture {
public:
  DracoControlArchitecture(RobotSystem *_robot);
  virtual ~DracoControlArchitecture();

  virtual void getCommand(void *_command);

  FootSE3TrajectoryManager *rfoot_tm;
  FootSE3TrajectoryManager *lfoot_tm;
  UpperBodyTrajectoryManager *upper_body_tm;
  FloatingBaseTrajectoryManager *floating_base_tm;
  DCMTrajectoryManager *dcm_tm;
  TaskHierarchyManager *rfoot_pos_hm;
  TaskHierarchyManager *rfoot_ori_hm;
  TaskHierarchyManager *lfoot_pos_hm;
  TaskHierarchyManager *lfoot_ori_hm;
  ReactionForceManager *rfoot_fm;
  ReactionForceManager *lfoot_fm;

  DracoTCIContainer *tci_container;

private:
  /* data */
  RobotSystem *robot_;
  DCMPlanner *dcm_planner_;

  DracoController *controller_;

  DracoStateProvider *sp_;
};
