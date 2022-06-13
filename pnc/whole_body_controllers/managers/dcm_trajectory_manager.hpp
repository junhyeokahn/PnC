#pragma once

#include "pnc/planners/locomotion/dcm_planner/dcm_planner.hpp"
#include "pnc/planners/locomotion/dcm_planner/footstep.hpp"
#include "pnc/robot_system/robot_system.hpp"
#include "pnc/whole_body_controllers/basic_tasks.hpp"

#include <g2o/core/optimizable_graph.h>
#include <muvt_core/optimizer/optimizer.h>
#include <muvt_core/environment/contact/vertex_contact.h>
#include <muvt_core/environment/contact/edge_collision.h>
#include <muvt_core/environment/contact/edge_relative_pose.h>
#include <muvt_core/environment/contact/edge_steering.h>

/// namespace dcm_transfer_type
namespace dcm_transfer_type {
constexpr int kInitial = 0;
constexpr int kMidStep = 1;
}; // namespace dcm_transfer_type

/// class DCMTrajectoryManager
class DCMTrajectoryManager {
public:
  /// \{ \name Constructor and Destructor
  DCMTrajectoryManager(DCMPlanner *_dcm_planner,
                       Task *_com_task, Task *_base_ori_task,
                       RobotSystem *_robot,
                       std::string _lfoot_idx, std::string _rfoot_idx);
  ~DCMTrajectoryManager();
  void paramInitialization(const YAML::Node &node);
  /// \}

  /// Initialize DCM planning.
  bool initialize(const double t_walk_start_in, const int transfer_type_in,
                  const Eigen::Quaterniond &ori_start_in,
                  const Eigen::Vector3d &dcm_pos_start_in,
                  const Eigen::Vector3d &dcm_vel_start_in);

  /// \{ \name Walking primitives
  /// Initialize trajectories for in-place walking.
  void walkInPlace();

  /// Initialize trajectories for forward walking.
  void walkForward();

  /// Initialize trajectories for backward walking.
  void walkBackward();

  /// Initialize trajectories for left walking.
  void strafeLeft();

  /// Initialize trajectories for right walking.
  void strafeRight();

  /// Initialize trajectories for counter clock wise turning.
  void turnLeft();

  /// Initialize trajectories for clock wise turning.
  void turnRight();
  /// \}

  /// Reset footstep lists.
  void resetIndexAndClearFootsteps();

  /// Alternate leg side.
  void alternateLeg();

  /// Updates the feet pose of the starting stance.
  void updateStartingStance();

  /// Return current footstep.
  int getStepIndex() { return current_footstep_idx; }

  /// Increment footstep index.
  void incrementStepIndex();

  /// Reset footstep index
  void resetStepIndex();

  /// Updates the local footstep list (ie: footstep preview) for trajectory
  /// generation.
  void updatePreview(const int max_footsteps_to_preview);

  /// Create footsteps for in place walking.
  void populateStepInPlace(const int num_steps, const int robot_side_first);

  /// Create footsteps for forward walking.
  void populateWalkForward(const int num_steps, const double forward_distance);

  /// Create footsteps for turning.
  void populateRotateTurn(const double turn_radians_per_step,
                          const int num_times);

  /// Create footsteps for side walking.
  void populateStrafe(const double strafe_distance, const int num_times);

  /// Local Footsteps Plan
  void localPlan();

  /// Save solution file
  void saveSolution(const std::string &);

  double t_walk_start_;
  Eigen::Quaterniond ori_start_;
  Footstep right_foot_start_;
  Footstep left_foot_start_;

  Footstep right_foot_stance_;
  Footstep left_foot_stance_;
  Footstep mid_foot_stance_;

  /// Footstep list.
  std::vector<Footstep> footstep_list;

  /// Index that keeps track of which footstep to take.
  int current_footstep_idx;

  double nominal_footwidth;
  double nominal_forward_step;
  double nominal_backward_step;
  double nominal_turn_radians;
  double nominal_strafe_distance;

  int robot_side_first_;

  /// Return total initial transfer time before the foot swing.
  double getInitialContactTransferTime();

  /// Return midstep transfer time before contact transition.
  double getMidStepContactTransferTime();

  /// Return final transfer time including settling time.
  double getFinalContactTransferTime();

  /// Return swing time.
  double getSwingTime();

  /// Return contact transition time.
  double getNormalForceRampUpTime();

  /// Return contact transition time.
  double getNormalForceRampDownTime();

  /// Return false if footstep_list is empty or current_step_index_ is greater
  /// than the footstep list and populate the robot_side when true.
  bool nextStepRobotSide(int &robot_side);

  /// Check whether or not there are remaining footsteps.
  bool noRemainingSteps();

  /// Update tasks command.
  void updateDCMTasksDesired(double current_time);

  int n_steps = 3;

private:

  void init_local_planner();

  RobotSystem *robot_;
  std::string lfoot_id_;
  std::string rfoot_id_;

  DCMPlanner *dcm_planner_;
  Muvt::HyperGraph::OptimizerContact *optimizer_;

  std::vector<Footstep> footstep_preview_list_;

  Task *com_task_;
  Task *base_ori_task_;

  Eigen::Vector3d des_dcm;
  Eigen::Vector3d des_dcm_vel;
  Eigen::Vector3d des_com_pos;
  Eigen::Vector3d des_com_vel;
  Eigen::Vector3d des_com_acc;
  Eigen::Quaternion<double> des_quat;
  Eigen::Vector3d des_ang_vel;
  Eigen::Vector3d des_ang_acc;

  /// \{ \name Human Readible Temporal Parameters
  /// Additional transfer time to swith the stance leg in the beginning.
  double t_additional_init_transfer_;

  /// Transition time used to change reaction force and stance leg.
  double t_contact_transition_;

  /// Foot swing time.
  double t_swing_;
  /// \}

  /// \{ \name DCM Planning Parameters
  /// Additional transfer time to swith the stance leg in the beginning.
  double t_transfer_init_;
  /// Additional transfer time to swith the stance leg in the mid step.
  double t_transfer_mid_;
  /// Double support polynomial transfer time.
  double t_ds_;
  /// Single support exponential interpolation time.
  double t_ss_;
  /// Percent to converge at the end of the trajectory .
  double percentage_settle_;
  /// Value between 0.0 and 1.0 for double support DCM interpolation.
  double alpha_ds_;
  /// \}

  double nominal_com_height_;

  /// Convert human readible temporal parameters to DCM planning parameters.
  void convertTemporalParamsToDCMParams();
};
