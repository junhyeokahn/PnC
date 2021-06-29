#pragma once

#include <Configuration.hpp>
#include <PnC/ControlArchitecture.hpp>
#include <PnC/Planner/Locomotion/DCMPlanner/DCMPlanner.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/WBC/Manager/DCMTrajectoryManager.hpp>
#include <PnC/WBC/Manager/FloatingBaseTrajectoryManager.hpp>
#include <PnC/WBC/Manager/FootSE3TrajectoryManager.hpp>
#include <PnC/WBC/Manager/ReactionForceManager.hpp>
#include <PnC/WBC/Manager/TaskHierarchyManager.hpp>
#include <PnC/WBC/Manager/UpperBodyTrajectoryManager.hpp>
#include <PnC/draco_pnc/draco_controller.hpp>
#include <PnC/draco_pnc/draco_tci_container.hpp>

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

private:
  /* data */
  RobotSystem *robot_;
  DCMPlanner *dcm_planner_;

  DracoController *controller_;
  DracoTCIContainer *tci_container_;

  DracoStateProvider *sp_;
};
