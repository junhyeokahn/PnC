#pragma once

#include <vector>

#include <PnC/A1PnC/A1Ctrl/A1MainController.hpp>
#include <PnC/A1PnC/A1Definition.hpp>
#include <PnC/ControlArchitecture.hpp>
// #include <PnC/A1PnC/A1StateMachine/ContactTransitionEnd.hpp>
// #include <PnC/A1PnC/A1StateMachine/ContactTransitionStart.hpp>
#include <PnC/A1PnC/A1StateMachine/QuadSupportBalance.hpp>
#include <PnC/A1PnC/A1StateMachine/QuadSupportStand.hpp>
// #include <PnC/A1PnC/A1StateMachine/SwingControl.hpp>
#include <PnC/A1PnC/A1StateProvider.hpp>
#include <PnC/A1PnC/A1TaskAndForceContainer/A1TaskAndForceContainer.hpp>
// #include <PnC/Planner/Footstep.hpp>
#include <PnC/TrajectoryManager/FloatingBaseTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/JointTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/MaxNormalForceTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/PointFootTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/TaskWeightTrajectoryManager.hpp>

namespace A1_STATES {
// constexpr int INITIALIZE = 0;
constexpr int STAND = 0;
constexpr int BALANCE = 1;
// constexpr int RL_CONTACT_TRANSITION_START = 3;
// constexpr int RL_CONTACT_TRANSITION_END = 4;
// constexpr int RL_SWING = 5;
// constexpr int LL_CONTACT_TRANSITION_START = 6;
// constexpr int LL_CONTACT_TRANSITION_END = 7;
// constexpr int LL_SWING = 8;
};  // namespace A1_STATES

class A1ControlArchitecture : public ControlArchitecture {
 public:
  A1ControlArchitecture(RobotSystem* _robot);
  virtual ~A1ControlArchitecture();
  virtual void ControlArchitectureInitialization();
  virtual void getCommand(void* _command);
  void saveData();

 protected:
  A1StateProvider* sp_;
  YAML::Node cfg_;

  void _InitializeParameters();
  bool b_state_first_visit_;

 public:
  // Task and Force Containers
  A1TaskAndForceContainer* taf_container_;
  // Controller Object
  A1MainController* main_controller_;
  // Add Planner

  // Trajectory Managers
  // JointTrajectoryManager* joint_trajectory_manager_; // If we want to keep
  // joint positions
  FloatingBaseTrajectoryManager*
      floating_base_lifting_up_manager_;  // if we want to manage COM height

  PointFootTrajectoryManager* frfoot_trajectory_manager_;
  PointFootTrajectoryManager* flfoot_trajectory_manager_;
  PointFootTrajectoryManager* rrfoot_trajectory_manager_;
  PointFootTrajectoryManager* rlfoot_trajectory_manager_;

  MaxNormalForceTrajectoryManager* flfoot_max_normal_force_manager_;
  MaxNormalForceTrajectoryManager* frfoot_max_normal_force_manager_;
  MaxNormalForceTrajectoryManager* rrfoot_max_normal_force_manager_;
  MaxNormalForceTrajectoryManager* rlfoot_max_normal_force_manager_;

  TaskWeightTrajectoryManager* flfoot_pos_hierarchy_manager_;
  TaskWeightTrajectoryManager* frfoot_pos_hierarchy_manager_;
  TaskWeightTrajectoryManager* rrfoot_pos_hierarchy_manager_;
  TaskWeightTrajectoryManager* rlfoot_pos_hierarchy_manager_;
  TaskWeightTrajectoryManager* com_hierarchy_manager_;
  TaskWeightTrajectoryManager* base_ori_hierarchy_manager_;
};
