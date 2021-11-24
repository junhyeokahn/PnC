#pragma once

#include "pnc/planners/locomotion/dcm_planner/dcm_planner.hpp"
#include "pnc/planners/locomotion/dcm_planner/footstep.hpp"
#include "pnc/robot_system/robot_system.hpp"
#include "pnc/whole_body_controllers/contact.hpp"
#include "pnc/whole_body_controllers/task.hpp"
#include "utils/interpolation.hpp"
#include "utils/util.hpp"

/// class FootTrajectoryManager
class FootTrajectoryManager {
public:
  /// \{ \name Constructor and Destructor
  FootTrajectoryManager(Task *_foot_pos_task, Task *_foot_ori_task,
                        RobotSystem *_robot);
  ~FootTrajectoryManager();
  /// \}

  /// Use current pose and vel to set zero acceleration command.
  void UpdateZeroAccCmd();

  void useNominalPoseCmd(const Eigen::Isometry3d& nominal_foot_iso); 

  /// Initialize the swing foot trajectory
  void InitializeSwingTrajectory(const double _start_time,
                                 const double _swing_duration,
                                 const Footstep &_landing_foot);

  /// computes the swing foot and updates the desired swing foot task
  void UpdateDesired(const double current_time);

  double swing_height;

private:
  void convertQuatDesToOriDes();
  RobotSystem *robot_;

  double start_time_;
  double duration_;

  Task *foot_pos_task_;
  Task *foot_ori_task_;
  std::string link_idx_;

  Footstep swing_init_foot_;
  Footstep swing_midfoot_;
  Footstep swing_land_foot_;

  HermiteCurveVec pos_traj_init_to_mid_;
  HermiteCurveVec pos_traj_mid_to_end_;
  HermiteQuaternionCurve quat_hermite_curve_;
};
