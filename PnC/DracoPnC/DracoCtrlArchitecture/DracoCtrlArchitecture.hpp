#pragma once

#include <vector>

#include <PnC/ControlArchitecture.hpp>
#include <PnC/DracoPnC/DracoCtrl/DracoMainController.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoStateMachine/ContactTransitionEnd.hpp>
#include <PnC/DracoPnC/DracoStateMachine/ContactTransitionStart.hpp>
#include <PnC/DracoPnC/DracoStateMachine/DoubleSupportBalance.hpp>
#include <PnC/DracoPnC/DracoStateMachine/DoubleSupportStand.hpp>
#include <PnC/DracoPnC/DracoStateMachine/Initialize.hpp>
#include <PnC/DracoPnC/DracoStateMachine/SwingControl.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/DracoTaskAndForceContainer/DracoTaskAndForceContainer.hpp>
#include <PnC/Planner/DCMPlanner.hpp>
#include <PnC/Planner/Footstep.hpp>
#include <PnC/TrajectoryManager/DCMTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/FloatingBaseTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/FootSE3TrajectoryManager.hpp>
#include <PnC/TrajectoryManager/JointTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/MaxNormalForceTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/TaskWeightTrajectoryManager.hpp>

namespace DRACO_STATES {
constexpr int INITIALIZE = 0;
constexpr int STAND = 1;
constexpr int BALANCE = 2;
constexpr int RL_CONTACT_TRANSITION_START = 3;
constexpr int RL_CONTACT_TRANSITION_END = 4;
constexpr int RL_SWING = 5;
constexpr int LL_CONTACT_TRANSITION_START = 6;
constexpr int LL_CONTACT_TRANSITION_END = 7;
constexpr int LL_SWING = 8;
};  // namespace DRACO_STATES

class DracoControlArchitecture : public ControlArchitecture {
 public:
  DracoControlArchitecture(RobotSystem* _robot);
  virtual ~DracoControlArchitecture();
  virtual void ControlArchitectureInitialization();
  virtual void getCommand(void* _command);
  void saveData();
  void getIVDCommand(void* _command);

 protected:
  DracoStateProvider* sp_;
  YAML::Node cfg_;

  void _InitializeParameters();
  bool b_state_first_visit_;

 public:
  // Task and Force Containers
  DracoTaskAndForceContainer* taf_container_;
  // Controller Object
  DracoMainController* main_controller_;
  // Add Planner
  DCMPlanner* dcm_planner_;

  // Trajectory Managers
  FootSE3TrajectoryManager* rfoot_trajectory_manager_;
  FootSE3TrajectoryManager* lfoot_trajectory_manager_;
  DCMTrajectoryManager* dcm_trajectory_manager_;
  JointTrajectoryManager* joint_trajectory_manager_;
  FloatingBaseTrajectoryManager* floating_base_lifting_up_manager_;
  MaxNormalForceTrajectoryManager* rfoot_front_max_normal_force_manager_;
  MaxNormalForceTrajectoryManager* rfoot_back_max_normal_force_manager_;
  MaxNormalForceTrajectoryManager* lfoot_front_max_normal_force_manager_;
  MaxNormalForceTrajectoryManager* lfoot_back_max_normal_force_manager_;
  TaskWeightTrajectoryManager* lfoot_pos_hierarchy_manager_;
  TaskWeightTrajectoryManager* lfoot_ori_hierarchy_manager_;
  TaskWeightTrajectoryManager* rfoot_pos_hierarchy_manager_;
  TaskWeightTrajectoryManager* rfoot_ori_hierarchy_manager_;
  TaskWeightTrajectoryManager* com_hierarchy_manager_;
  TaskWeightTrajectoryManager* base_ori_hierarchy_manager_;
};
