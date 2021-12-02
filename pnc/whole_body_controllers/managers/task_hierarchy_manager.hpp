#pragma once

#include "pnc/whole_body_controllers/task.hpp"

/// class TaskHierarchyManager
class TaskHierarchyManager {
public:
  /// \{ \name Constructor and Destructor
  TaskHierarchyManager(Task *_task, Eigen::VectorXd _w_max,
                       Eigen::VectorXd _w_min);
  ~TaskHierarchyManager();
  /// \}

  /// Initialize hierarchy transition to min
  void InitializeRampToMin(double _start_time, double _duration);

  /// Initialize hierarchy transition to max
  void InitializeRampToMax(double _start_time, double _duration);

  /// Update task hierarchy weight
  void UpdateRampToMin(double _current_time);

  /// Update task hierarchy weight
  void UpdateRampToMax(double _current_time);

protected:
  Task *task_;
  Eigen::VectorXd w_max_;
  Eigen::VectorXd w_min_;
  Eigen::VectorXd w_starting_;
  double start_time_;
  double duration_;
};
