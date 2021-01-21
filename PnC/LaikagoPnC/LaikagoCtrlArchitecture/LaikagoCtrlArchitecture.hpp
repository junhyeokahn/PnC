#pragma once

#include <vector>

#include <PnC/ControlArchitecture.hpp>
#include <PnC/LaikagoPnC/LaikagoCtrl/LaikagoMainController.hpp>
#include <PnC/LaikagoPnC/LaikagoDefinition.hpp>
#include <PnC/LaikagoPnC/LaikagoStateMachine/ContactTransitionEnd.hpp>
#include <PnC/LaikagoPnC/LaikagoStateMachine/ContactTransitionStart.hpp>
#include <PnC/LaikagoPnC/LaikagoStateMachine/DoubleSupportBalance.hpp>
#include <PnC/LaikagoPnC/LaikagoStateMachine/DoubleSupportStand.hpp>
#include <PnC/LaikagoPnC/LaikagoStateMachine/Initialize.hpp>
#include <PnC/LaikagoPnC/LaikagoStateMachine/SwingControl.hpp>
#include <PnC/LaikagoPnC/LaikagoStateProvider.hpp>
#include <PnC/LaikagoPnC/LaikagoTaskAndForceContainer/LaikagoTaskAndForceContainer.hpp>
#include <PnC/Planner/DCMPlanner.hpp>
#include <PnC/Planner/Footstep.hpp>
#include <PnC/TrajectoryManager/DCMTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/FloatingBaseTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/FootSE3TrajectoryManager.hpp>
#include <PnC/TrajectoryManager/JointTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/MaxNormalForceTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/TaskWeightTrajectoryManager.hpp>

namespace Laikago_STATES {
constexpr int INITIALIZE = 0;
constexpr int STAND = 1;
constexpr int BALANCE = 2;
constexpr int RL_CONTACT_TRANSITION_START = 3;
constexpr int RL_CONTACT_TRANSITION_END = 4;
constexpr int RL_SWING = 5;
constexpr int LL_CONTACT_TRANSITION_START = 6;
constexpr int LL_CONTACT_TRANSITION_END = 7;
constexpr int LL_SWING = 8;
};  // namespace Laikago_STATES

class LaikagoControlArchitecture : public ControlArchitecture {
 public:
  LaikagoControlArchitecture(RobotSystem* _robot);
  virtual ~LaikagoControlArchitecture();
  virtual void ControlArchitectureInitialization();
  virtual void getCommand(void* _command);
  void saveData();
  void getIVDCommand(void* _command);
  void smoothing_torque(void* _cmd);

 protected:
  LaikagoStateProvider* sp_;
  YAML::Node cfg_;

  void _InitializeParameters();
  bool b_state_first_visit_;

 public:
  // Task and Force Containers
  LaikagoTaskAndForceContainer* taf_container_;
  // Controller Object
  LaikagoMainController* main_controller_;
  // Add Planner
  DCMPlanner* dcm_planner_;

  // Trajectory Managers
  FootSE3TrajectoryManager* rfoot_trajectory_manager_;
  FootSE3TrajectoryManager* lfoot_trajectory_manager_;
  DCMTrajectoryManager* dcm_trajectory_manager_;
  JointTrajectoryManager* joint_trajectory_manager_;
  FloatingBaseTrajectoryManager* floating_base_lifting_up_manager_;
  // MaxNormalForceTrajectoryManager* rfoot_front_max_normal_force_manager_;
  // MaxNormalForceTrajectoryManager* rfoot_back_max_normal_force_manager_;
  // MaxNormalForceTrajectoryManager* lfoot_front_max_normal_force_manager_;
  // MaxNormalForceTrajectoryManager* lfoot_back_max_normal_force_manager_;
  MaxNormalForceTrajectoryManager* lfoot_max_normal_force_manager_;
  MaxNormalForceTrajectoryManager* rfoot_max_normal_force_manager_;
  TaskWeightTrajectoryManager* lfoot_pos_hierarchy_manager_;
  TaskWeightTrajectoryManager* lfoot_ori_hierarchy_manager_;
  TaskWeightTrajectoryManager* rfoot_pos_hierarchy_manager_;
  TaskWeightTrajectoryManager* rfoot_ori_hierarchy_manager_;
  TaskWeightTrajectoryManager* com_hierarchy_manager_;
  TaskWeightTrajectoryManager* base_ori_hierarchy_manager_;
};
