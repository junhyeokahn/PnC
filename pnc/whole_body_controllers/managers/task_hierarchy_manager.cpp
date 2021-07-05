#include <pnc/whole_body_controllers/managers/task_hierarchy_manager.hpp>

TaskHierarchyManager::TaskHierarchyManager(Task *_task, double _w_max,
                                           double _w_min) {
  util::PrettyConstructor(2, "TaskHierarchyManager");

  task_ = _task;
  w_max_ = _w_max;
  w_min_ = _w_min;
  w_starting_ = task_->w_hierarchy;
  start_time_ = 0.;
  duration_ = 0.;
}

TaskHierarchyManager::~TaskHierarchyManager() {}

void TaskHierarchyManager::InitializeRampToMin(double _start_time,
                                               double _duration) {
  start_time_ = _start_time;
  duration_ = _duration;
  w_starting_ = task_->w_hierarchy;
}

void TaskHierarchyManager::InitializeRampToMax(double _start_time,
                                               double _duration) {
  start_time_ = _start_time;
  duration_ = _duration;
  w_starting_ = task_->w_hierarchy;
}

void TaskHierarchyManager::UpdateRampToMin(double _curr_time) {
  double t = util::Clamp(_curr_time, start_time_, start_time_ + duration_);
  task_->w_hierarchy =
      (w_min_ - w_starting_) / duration_ * (t - start_time_) + w_starting_;
}

void TaskHierarchyManager::UpdateRampToMax(double _curr_time) {
  double t = util::Clamp(_curr_time, start_time_, start_time_ + duration_);
  task_->w_hierarchy =
      (w_max_ - w_starting_) / duration_ * (t - start_time_) + w_starting_;
}
