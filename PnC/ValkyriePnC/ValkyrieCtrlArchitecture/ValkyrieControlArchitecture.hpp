#pragma once

#include <vector>

#include <PnC/ControlArchitecture.hpp>
#include <PnC/Planner/DCMPlanner.hpp>
#include <PnC/Planner/Footstep.hpp>
#include <PnC/TrajectoryManager/DCMTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/FootSE3TrajectoryManager.hpp>
#include <PnC/TrajectoryManager/MaxNormalForceTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/TaskWeightTrajectoryManager.hpp>
#include <PnC/ValkyriePnC/ValkyrieCtrl/ValkyrieMainController.hpp>
#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateMachine/ContactTransition.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateMachine/ContactTransitionEnd.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateMachine/DoubleSupportBalance.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateMachine/DoubleSupportStand.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateMachine/SwingControl.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>
#include <PnC/ValkyriePnC/ValkyrieTaskAndForceContainer/ValkyrieTaskAndForceContainer.hpp>
#include <PnC/ValkyriePnC/ValkyrieTrajectoryManager/UpperBodyJointTrajectoryManager.hpp>

namespace VALKYRIE_STATES {
constexpr int STAND = 0;
constexpr int BALANCE = 1;
constexpr int RL_CONTACT_TRANSITION_START = 2;
constexpr int RL_CONTACT_TRANSITION_END = 3;
constexpr int RL_SWING = 4;
constexpr int LL_CONTACT_TRANSITION_START = 5;
constexpr int LL_CONTACT_TRANSITION_END = 6;
constexpr int LL_SWING = 7;
};

class ValkyrieControlArchitecture : public ControlArchitecture {
 public:
  ValkyrieControlArchitecture(RobotSystem* _robot);
  virtual ~ValkyrieControlArchitecture();
  virtual void ControlArchitectureInitialization();
  virtual void getCommand(void* _command);

 protected:
  ValkyrieStateProvider* sp_;
  YAML::Node cfg_;

  void _InitializeParameters();
  bool b_state_first_visit_;

 public:
  // Task and Force Containers
  ValkyrieTaskAndForceContainer* taf_container_;
  // Controller Object
  ValkyrieMainController* main_controller_;
  // Add Planner
  DCMPlanner* dcm_planner_;

  // Trajectory Managers
  FootSE3TrajectoryManager* rfoot_trajectory_manager_;
  FootSE3TrajectoryManager* lfoot_trajectory_manager_;
  MaxNormalForceTrajectoryManager* lfoot_max_normal_force_manager_;
  MaxNormalForceTrajectoryManager* rfoot_max_normal_force_manager_;
  UpperBodyJointTrajectoryManager* upper_body_joint_trajectory_manager_;
  TaskWeightTrajectoryManager* lfoot_contact_pos_hierarchy_manager_;
  TaskWeightTrajectoryManager* lfoot_contact_ori_hierarchy_manager_;
  TaskWeightTrajectoryManager* rfoot_contact_pos_hierarchy_manager_;
  TaskWeightTrajectoryManager* rfoot_contact_ori_hierarchy_manager_;
  DCMTrajectoryManager* dcm_trajectory_manger_;
};
