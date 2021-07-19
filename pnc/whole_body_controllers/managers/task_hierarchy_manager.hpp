#pragma once

#include "pnc/whole_body_controllers/task.hpp"

/// class TaskHierarchyManager
class TaskHierarchyManager {
public:
  /// \{ \name Constructor and Destructor
  TaskHierarchyManager(Task *_task, double _w_max, double _w_min);
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
  double w_max_;
  double w_min_;
  double w_starting_;
  double start_time_;
  double duration_;
};
