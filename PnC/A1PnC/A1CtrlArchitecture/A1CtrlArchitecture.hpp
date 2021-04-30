#pragma once

#include <vector>

#include <PnC/A1PnC/A1Ctrl/A1MainController.hpp>
#include <PnC/A1PnC/A1Definition.hpp>
#include <PnC/ControlArchitecture.hpp>
#include <PnC/A1PnC/A1StateMachine/ContactTransitionEnd.hpp>
#include <PnC/A1PnC/A1StateMachine/ContactTransitionStart.hpp>
#include <PnC/A1PnC/A1StateMachine/QuadSupportBalance.hpp>
#include <PnC/A1PnC/A1StateMachine/QuadSupportStand.hpp>
#include <PnC/A1PnC/A1StateMachine/SwingControl.hpp>
#include <PnC/A1PnC/A1StateProvider.hpp>
#include <PnC/A1PnC/A1TaskAndForceContainer/A1TaskAndForceContainer.hpp>
// #include <PnC/ConvexMPC/ConvexMPC.hpp>
#include <PnC/TrajectoryManager/FloatingBaseTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/JointTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/MaxNormalForceTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/PointFootTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/TaskWeightTrajectoryManager.hpp>
#include <PnC/TrajectoryManager/ReactionForceTrajectoryManager.hpp>
#include <math.h>

namespace A1_STATES {
// constexpr int INITIALIZE = 0;
constexpr int STAND = 0;
constexpr int BALANCE = 1;
constexpr int FL_CONTACT_TRANSITION_START = 2;
constexpr int FL_CONTACT_TRANSITION_END = 3;
constexpr int FL_SWING = 4;
constexpr int FR_CONTACT_TRANSITION_START = 5;
constexpr int FR_CONTACT_TRANSITION_END = 6;
constexpr int FR_SWING = 7;
};  // namespace A1_STATES

class A1ControlArchitecture : public ControlArchitecture {
 public:
  A1ControlArchitecture(RobotSystem* _robot);
  virtual ~A1ControlArchitecture();
  virtual void ControlArchitectureInitialization();
  virtual void getCommand(void* _command);
  void saveData();
  void solveMPC();

  Eigen::VectorXd command_rxn_forces;

 protected:
  A1StateProvider* sp_;
  YAML::Node cfg_;

  void saveMPCSolution(const Eigen::VectorXd com_pos,
                       const Eigen::VectorXd com_vel_body_frame,
                       const Eigen::VectorXd com_rpy_zyx,
                       const Eigen::VectorXd ang_vel,
                       const Eigen::VectorXd foot_pos_body_frame);

  void _InitializeParameters();
  bool b_state_first_visit_;
  int state_holder_for_mpc_;

 public:
  // Task and Force Containers
  A1TaskAndForceContainer* taf_container_;
  // Controller Object
  A1MainController* main_controller_;
  // MPC Planner
  ConvexMPC* mpc_planner_;

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

  ReactionForceTrajectoryManager* rxn_force_manager_;

  private:
  int num_mpc_calls;
  double mass = 9.713 + 0.5*(0.696 + 1.013 + 0.166 + 0.06) * 4;
  int num_legs = 4;
  int _PLANNING_HORIZON_STEPS = 10;
  double _PLANNING_TIMESTEP = 0.025; // WBC running at .002 // We call MPC every .012
  Eigen::VectorXd body_inertia;
  Eigen::VectorXd _MPC_WEIGHTS;
  Eigen::VectorXi foot_contact_states;
  Eigen::VectorXd foot_pos_body_frame;
  Eigen::VectorXd foot_friction_coeffs;
  int mpc_counter;
  Eigen::Vector3d frfoot_body_frame, flfoot_body_frame,
                  rlfoot_body_frame, rrfoot_body_frame;

};
