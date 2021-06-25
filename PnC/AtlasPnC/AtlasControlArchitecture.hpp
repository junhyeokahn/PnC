#pragma once

#include <PnC/AtlasPnC/AtlasController.hpp>
#include <PnC/AtlasPnC/AtlasTCIContainer.hpp>
#include <PnC/ControlArchitecture.hpp>
#include <PnC/Planner/Locomotion/DCMPlanner/DCMPlanner.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/WBC/Manager/DCMTrajectoryManager.hpp>
#include <PnC/WBC/Manager/FloatingBaseTrajectoryManager.hpp>
#include <PnC/WBC/Manager/FootSE3TrajectoryManager.hpp>
#include <PnC/WBC/Manager/ReactionForceManager.hpp>
#include <PnC/WBC/Manager/TaskHierarchyManager.hpp>
#include <PnC/WBC/Manager/UpperBodyTrajectoryManager.hpp>

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

  AtlasController *controller_;
  AtlasTCIContainer *tci_container_;

  AtlasStateProvider *sp_;
};
