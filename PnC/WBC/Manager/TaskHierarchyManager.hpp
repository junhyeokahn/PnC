#pragma once

#include <PnC/WBC/Task.hpp>
#include <Utils/Math/MathUtilities.hpp>

class TaskHierarchyManager {
public:
  TaskHierarchyManager(Task *_task, double _w_max, double _w_min);
  ~TaskHierarchyManager();

  void initialize_ramp_to_min(double _start_time, double _duration);
  void initialize_ramp_to_max(double _start_time, double _duration);
  void update_ramp_to_min(double _current_time);
  void update_ramp_to_max(double _current_time);

protected:
  Task *task_;
  double w_max_;
  double w_min_;
  double w_starting_;
  double start_time_;
  double duration_;
};
