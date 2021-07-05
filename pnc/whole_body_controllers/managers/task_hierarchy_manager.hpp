#pragma once

#include <pnc/whole_body_controllers/task.hpp>

class TaskHierarchyManager {
public:
  TaskHierarchyManager(Task *_task, double _w_max, double _w_min);
  ~TaskHierarchyManager();

  void InitializeRampToMin(double _start_time, double _duration);
  void InitializeRampToMax(double _start_time, double _duration);
  void UpdateRampToMin(double _current_time);
  void UpdateRampToMax(double _current_time);

protected:
  Task *task_;
  double w_max_;
  double w_min_;
  double w_starting_;
  double start_time_;
  double duration_;
};
